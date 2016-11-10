function [input,navMemory,txMsg] = simNavDecision(currentMeasure,navMemory,Nagents,aa,rxMsgs,t,dt,kk,number_in_cloud)
%------------------------------------------------------------------
% DEFAULT TRANSMIT MESSAGE
%------------------------------------------------------------------
txMsg = [];
currentMeasure.ownPosition = currentMeasure.ownPosition;

%------------------------------------------------------------------
% DETERMINE IF NEXT UAV SHOULD BE LAUNCHED
%------------------------------------------------------------------
activeCounter = 0;
readyToDeploy = 0;
% 0 = Not Active, 1 = Track Mode, 2 = Return Flag Turned On, 3 = Newly Active Agent, 4 = Parked
for ii = 1 : numel(rxMsgs)
    if rxMsgs{ii}(4) ~= 0
        activeCounter = activeCounter + 1;
    end
    if rxMsgs{ii}(4) == 2
        readyToDeploy = 1;
    end
end
for ii = 1 : numel(rxMsgs)
    if rxMsgs{ii}(4) == 0 && readyToDeploy == 1
        if navMemory.number == ii
            navMemory.newlyActive = 1;
            navMemory.Active = 1;
            navMemory.parked = 0;
            break
        end
        break
    end
end
for ii = 1 : numel(rxMsgs)
    if rxMsgs{ii}(4) == 3 && ii ~= navMemory.number
        navMemory.Active_Agents = navMemory.Active_Agents + 1;
    end
end
if navMemory.newlyActive == 1
    navMemory.Active_Agents = navMemory.number;
    navMemory.initialCounter = 1;
end
if navMemory.initialCounter > 0 && navMemory.initialCounter < 7
    navMemory.initialCounter = navMemory.initialCounter + 1;
end
if navMemory.initialCounter >= 6
    navMemory.initialCounter = 0;
end

%------------------------------------------------------------------
% IF AGENT NOT ACTIVE DON'T RUN CODE
%------------------------------------------------------------------
% if navMemory.parked == 1 
%     input(1) = 0;
%     input(2) = 0;
%     return
% end
    
%------------------------------------------------------------------
% DETERMINE OBSTACLES
%------------------------------------------------------------------
nearestDist = inf;
currentMeasure.nearestObs = [0;0];
for mm = 1 : numel(rxMsgs)
    if mm ~= navMemory.number && rxMsgs{mm}(4) ~= 0
        thisDist = norm(currentMeasure.ownPosition - rxMsgs{mm}(1:2));
        if (thisDist<nearestDist),
            currentMeasure.nearestObs = rxMsgs{mm}(1:2);
            nearestDist = thisDist;
        end
    end
end

%------------------------------------------------------------------
% CALCULATE HEADING
%------------------------------------------------------------------
navMemory.headEstimate = atan2d( (currentMeasure.ownPosition(2) - navMemory.lastPos(2)), ...
    (currentMeasure.ownPosition(1) - navMemory.lastPos(1)));

%------------------------------------------------------------------
% RESET MEMORY WITH NEW MEASUREMENTS
%------------------------------------------------------------------
navMemory.error(kk) = (currentMeasure.p - 1);
navMemory.error_005(kk) = (currentMeasure.p - 0.1);
if kk > 1
    navMemory.dp(kk) = (navMemory.error(kk) - navMemory.error(kk-1))/dt;
    navMemory.dp_005(kk) = (navMemory.error_005(kk) - navMemory.error_005(kk-1))/dt;
end

%------------------------------------------------------------------
% BUILD CONCENTRATION MAP
%------------------------------------------------------------------
for mm=1:numel(rxMsgs),
    if rxMsgs{mm}(3) > 0.2
        x = round(rxMsgs{mm}(1)+1000);
        y = round(rxMsgs{mm}(2)+1000);
        navMemory.concMap(x,y) = rxMsgs{mm}(3);
    end
end

%------------------------------------------------------------------
% BUILD MAP OF UAVS POSITIONS & DETERMINE CLOSEST UAV TO BASE
%------------------------------------------------------------------
nearestDist = inf;
for mm = 1 : numel(rxMsgs)
    navMemory.xPositionStore(:,mm,kk) = rxMsgs{mm}(1);
    navMemory.yPositionStore(:,mm,kk) = rxMsgs{mm}(2);
    if norm([navMemory.xPositionStore(:,mm,kk);navMemory.yPositionStore(:,mm,kk)]) <...
            nearestDist && rxMsgs{mm}(4) ~= 0 && rxMsgs{mm}(4) ~= 4
        nearestDist = norm([navMemory.xPositionStore(:,mm,kk);navMemory.yPositionStore(:,mm,kk)]);
        closestAgaentToBase = mm;
    end
end


%------------------------------------------------------------------
% CALCULATE PERIMETER OF CLOUD FROM CONCENTRATION MAP
%------------------------------------------------------------------
if max(max(navMemory.concMap)) > 0
    [x_targ_pos_1 y_targ_pos_1] = find(abs(navMemory.concMap) >= 1);
    shp = alphaShape(x_targ_pos_1, y_targ_pos_1);
    navMemory.cloudPerimeter = perimeter(shp);
    if number_in_cloud ~= 0 && navMemory.cloudPerimeter > 1
        cloudFitNumber = navMemory.cloudPerimeter / number_in_cloud;
    end
    navMemory.cloudCentre = [(min(x_targ_pos_1) + (max(x_targ_pos_1) - min(x_targ_pos_1))/2 - 1000);...
        ((min(y_targ_pos_1) + (max(y_targ_pos_1) - min(y_targ_pos_1))/2 - 1000))];
end

%------------------------------------------------------------------
% CALCUALTE IF THERE ARE UAVS IN FRONT OR BEHIND
%------------------------------------------------------------------
% Area In Front of UAV
rotationAngle = atan2( (currentMeasure.ownPosition(1) - navMemory.lastPos(1)), ...
    (currentMeasure.ownPosition(2) - navMemory.lastPos(2)));
rotationArray = [cos(rotationAngle), -sin(rotationAngle); sin(rotationAngle), cos(rotationAngle)]';
rectangle = [-100 -100 100 100; 0 200 200 0];
rotated_rectangle = rotationArray * rectangle;
x1 = currentMeasure.ownPosition + rotated_rectangle(:,1);
x2 = currentMeasure.ownPosition + rotated_rectangle(:,2);
x3 = currentMeasure.ownPosition + rotated_rectangle(:,3);
x4 = currentMeasure.ownPosition + rotated_rectangle(:,4);
navMemory.inFrontX = [x1(1) x2(1) x3(1) x4(1)];
navMemory.inFrontY = [x1(2) x2(2) x3(2) x4(2)];

% Area Behind of UAV
x1 = currentMeasure.ownPosition - rotated_rectangle(:,1);
x2 = currentMeasure.ownPosition - rotated_rectangle(:,2);
x3 = currentMeasure.ownPosition - rotated_rectangle(:,3);
x4 = currentMeasure.ownPosition - rotated_rectangle(:,4);
navMemory.behindX = [x1(1) x2(1) x3(1) x4(1)];
navMemory.behindY = [x1(2) x2(2) x3(2) x4(2)];

% Set Flag For UAVs To Speed Up or Slow Down
count=0;
navMemory.speedUpFlag = 0;
navMemory.slowDownFlag = 0;

navMemory.uav_counter = 0;
if numel(rxMsgs) > 0
    for mm = 1 : numel(rxMsgs)
        if rxMsgs{mm}(3) > 0.5 && rxMsgs{mm}(4) ~= 0
            navMemory.uav_counter = navMemory.uav_counter + 1;
        end
        if mm ~= aa && rxMsgs{mm}(4) ~= 0
            if count == 0
                xq = [rxMsgs{mm}(1)];
                yq = [rxMsgs{mm}(2)];
                count = 1;
                count2 = 0;
            end
            if count >= 1 && count2 >= 1
                xq = [xq rxMsgs{mm}(1)];
                yq = [yq rxMsgs{mm}(2)];
            end
            count2 = 1;
        end
    end
end

if navMemory.trackMode == 1 && numel(xq) ~= 0 && numel(yq) ~= 0 
    in_front = inpolygon(xq,yq,navMemory.inFrontX,navMemory.inFrontY);
    for ii = 1 : numel(in_front)
        if in_front(ii) == 1
            navMemory.slowDownFlag = 1;
        end
    end
    in_behind = inpolygon(xq,yq,navMemory.behindX,navMemory.behindY);
    for ii = 1 : numel(in_behind)
        if in_behind(ii) == 1
            navMemory.speedUpFlag = 1;
        end
    end
end

%------------------------------------------------------------------
% CALCUALTE IF THERE ARE UAVS IN FRONT FOR COLLISION AVOIDANCE
%------------------------------------------------------------------
rectangle = [-200 -200 200 200; -100 100 100 -100];
rotated_rectangle = rotationArray * rectangle;
x1 = currentMeasure.ownPosition + rotated_rectangle(:,1);
x2 = currentMeasure.ownPosition + rotated_rectangle(:,2);
x3 = currentMeasure.ownPosition + rotated_rectangle(:,3);
x4 = currentMeasure.ownPosition + rotated_rectangle(:,4);
navMemory.FrontRightX = [x1(1) x2(1) x3(1) x4(1)];
navMemory.FrontRightY = [x1(2) x2(2) x3(2) x4(2)];

% Set Flag For UAVs To Speed Up or Slow Down
count=0;
navMemory.CollisionRight = 0;
if navMemory.trackMode ~= 1 && numel(rxMsgs) > 0 && navMemory.avoidanceCounter ~= 1
    front_right = inpolygon(xq,yq,navMemory.FrontRightX,navMemory.FrontRightY);
    for ii = 1 : numel(front_right)
        if front_right(ii) == 1
            navMemory.CollisionRight = 1;
        end
    end
end


%------------------------------------------------------------------
% CALCULATE POINT FOR UAVS TO FLY TO
%------------------------------------------------------------------
[x_targ_pos y_targ_pos] = find(abs(navMemory.concMap-1) ...
    == min(min(abs(navMemory.concMap-1))),1);
x_targ_pos = x_targ_pos - 1000;
y_targ_pos = y_targ_pos - 1000;


%------------------------------------------------------------------
% DETERMINE IF UAV SHOULD FLY TO CLOUD 
%------------------------------------------------------------------          
nearestDist = inf;
closest_robot = 0;
counter = 0;
navMemory.cloudReadyForNextUAV = 0;
if navMemory.trackMode ~= 1;
    for mm = 1 : numel(rxMsgs)
        if rxMsgs{mm}(3) > 0.5 && rxMsgs{mm}(4) ~= 0
            for ll = 1 : numel(rxMsgs)
                if rxMsgs{ll}(3) < 0.1 && rxMsgs{ll}(4) ~= 0
                    Dist = norm([x_targ_pos;y_targ_pos] - rxMsgs{ll}(1:2));
                    if (Dist<nearestDist),
                        nearestDist = Dist;
                        closest_robot = ll;
                    end
                end
            end
        end
    end
end

% Are Any UAVs Too Close In Cloud
AllUavsnearestDist = inf;
for mm = 1 : numel(rxMsgs)
    if rxMsgs{mm}(3) > 0.1 && rxMsgs{mm}(4) ~= 0
        for ll = 1 : numel(rxMsgs)
            if rxMsgs{ll}(3) > 0.1 && rxMsgs{mm}(4) ~= 0
                if mm ~= ll
                    AllUavsDist = norm(rxMsgs{mm}(1:2) - rxMsgs{ll}(1:2));
                    if (AllUavsDist<AllUavsnearestDist),
                        AllUavsnearestDist = AllUavsDist;
                    end
                end
            end
        end
    end
end

if closest_robot == aa && AllUavsnearestDist > 100
    navMemory.flyingToCloud = 1;
end



%------------------------------------------------------------------
% PRE FSM CHECKS
%------------------------------------------------------------------
if navMemory.newlyActive == 1 && navMemory.CollisionRight ~= 1
    navMemory.navState = 10;
elseif navMemory.trackMode == 1 && abs(currentMeasure.ownPosition(1)) > 800 ...
        || navMemory.trackMode == 1 && abs(currentMeasure.ownPosition(2)) > 800
    navMemory.navState = 12;
elseif navMemory.Active == 0 || navMemory.parked == 1
    navMemory.navState = 1;
elseif navMemory.returning == 1
    if abs(currentMeasure.ownPosition(1)) < 50 && abs(currentMeasure.ownPosition(2)) < 50
        navMemory.navState = 1;
        navMemory.parked = 1;
    end
    navMemory.deployUAV = 0;
elseif t < 54 %navMemory.spreadCounter < 14 && kk < 15
    navMemory.navState = 2;
elseif abs(currentMeasure.ownPosition(1)) > 900 || abs(currentMeasure.ownPosition(2)) > 900
    % Check if UAV is approaching boundary
    navMemory.navState = 9;
elseif navMemory.CollisionRight == 1 && navMemory.initialCounter == 0
    navMemory.navState = 7;
elseif navMemory.flightTime + navMemory.number*100 > 1500  && navMemory.number <= Nagents && ...
        navMemory.number == closestAgaentToBase
    % Return UAVs to hanger before 30 minutes is up
    navMemory.navState = 11;
elseif navMemory.flightTime + navMemory.number*100 > 2100  && navMemory.number > Nagents && ...
        navMemory.number == closestAgaentToBase
    % Return UAVs to hanger before 30 minutes is up
    navMemory.navState = 11;
elseif navMemory.flightTime + navMemory.number*100 > 2100  && navMemory.number > Nagents && ...
        navMemory.number == closestAgaentToBase
    % Return UAVs to hanger before 30 minutes is up
    navMemory.navState = 11;
elseif navMemory.slowDownFlag == 1;
    navMemory.navState = 6;
elseif navMemory.speedUpFlag == 1;
    navMemory.navState = 5;
elseif currentMeasure.p > 0.6
    navMemory.navState = 4;
elseif navMemory.flyingToCloud == 1
    navMemory.navState = 10;
elseif navMemory.newlyActive == 1 && navMemory.Active == 1
    navMemory.navState = 10;
end    


%------------------------------------------------------------------
% FINITE STATE MACHINE
%------------------------------------------------------------------
switch navMemory.navState
    
    case 1, % State 1 - Waiting to be launched
        input(1) = 0;
        input(2) = 0;
        navMemory.trackMode = 0;
        navMemory.parked = 1;
        navMemory.avoidanceCounter = 0;
    
    case 2, % State 2 - Spread agents
        input(1) = 10;
        input(2) = 0;
        navMemory.spreadCounter = navMemory.spreadCounter + 1;
        if navMemory.spreadCounter > 15
            navMemory.navState = 3;
        end
        navMemory.trackMode = 0;
        navMemory.avoidanceCounter = 0;

    case 3, % State 3 - Spiral Agents (Search Mode)
        input(1) = 10;
        input(2) = 20/(t/3.6);
        if currentMeasure.p > 0.9
            navMemory.navState = 4;
        end
        navMemory.trackMode = 0;
        navMemory.avoidanceCounter = 0;
        
    case 4, % State 4 - Track Cloud
        input(1) = 12;
        input(2) = (10*navMemory.dp(kk) + 1*navMemory.error(kk));
        navMemory.trackMode = 1;
        navMemory.avoidanceCounter = 0;
        
    case 5, % State 5 - Speed Up If UAV Behind
        input(1) = 14;
        input(2) = (10*navMemory.dp(kk) + 1*navMemory.error(kk));
        navMemory.trackMode = 1;
        navMemory.speedUpFlag = 0;
        navMemory.avoidanceCounter = 0;
        
    case 6, % State 6 - Slow Down If UAV In Front
        input(1) = 10;
        input(2) = (20*navMemory.dp(kk) + 1*navMemory.error(kk));
        navMemory.trackMode = 1;
        navMemory.slowDownFlag = 0;
        navMemory.avoidanceCounter = 0;
   
    case 7 % State 7 - Collision Avoidance (TurnAround)
        input(1) = 10;
        input(2) = 180/(input(1)*dt);
        navMemory.navState = 8;
        navMemory.avoidanceCounter = 1;
        navMemory.trackMode = 0;  
        
    case 8 % State 8 - Collision Avoidance (Move Away)
        input(1) = 10;
        input(2) = 0;
        navMemory.avoidanceCounter = 1;
        navMemory.trackMode = 0;
        
    case 9, % State 9 - Prevent UAV from going out-of-bounds by flying towards base
        input(1) = 10;
        base_position = [0;0];
        
        % Turning Controller
        navMemory.returningError(kk) = navMemory.headEstimate - atan2d(base_position(2)...
            -currentMeasure.ownPosition(2),base_position(1)-currentMeasure.ownPosition(1));
        derivativeReturningError = (navMemory.returningError(kk) - navMemory.returningError(kk-1))/dt;
        input(2) = (0.5*navMemory.returningError(kk) + 0.5*derivativeReturningError)/(input(1)*dt);
        
        navMemory.trackMode = 0;
        navMemory.avoidanceCounter = 0;
        if abs(currentMeasure.ownPosition(1)) < 50 || ...
                abs(currentMeasure.ownPosition(2)) < 50
            input(2) = 0.2;
        end
        
    case 10, %State 10 - Move towards UAV in cloud
        navMemory.swarmPosition = [x_targ_pos; y_targ_pos];
        input(1) = 10;
        
        % Turning Controller
        navMemory.turningError(kk) = navMemory.headEstimate - atan2d((navMemory.swarmPosition(2)...
            - currentMeasure.ownPosition(2)),(navMemory.swarmPosition(1) - ...
            currentMeasure.ownPosition(1)));
        derivativeTurningError = (navMemory.turningError(kk) - navMemory.turningError(kk-1))/dt;
        input(2) = (0.5*navMemory.turningError(kk) + 0.5*derivativeTurningError)/(input(1)*dt);
        
        if abs(currentMeasure.ownPosition(1)) < 50 || ...
                abs(currentMeasure.ownPosition(2)) < 50
            input(2) = 0.2;
        end
        
        navMemory.trackMode = 0;
        navMemory.flyingToCloud = 0;
        navMemory.newlyActive = 0;
        navMemory.parked = 0;
        navMemory.avoidanceCounter = 0;
        
    case 11, % State 11 - Return to base
        input(1) = 10;
        base_position = [0;0];
        % Turning Controller
        navMemory.returningError(kk) = navMemory.headEstimate - atan2d(base_position(2)-...
            currentMeasure.ownPosition(2),base_position(1)-currentMeasure.ownPosition(1));
        derivativeReturningError = (navMemory.returningError(kk) - navMemory.returningError(kk-1))/dt;
        input(2) = (0.5*navMemory.returningError(kk) + 0.5*derivativeReturningError)/(input(1)*dt);
        
        if abs(currentMeasure.ownPosition(1)) < 50 && abs(currentMeasure.ownPosition(2)) < 50
            navMemory.navState = 1;
            input(1) = 0;
            input(2) = 0;
            navMemory.deployUAV = 0;
            navMemory.parked = 1;
        end
        navMemory.deployUAV = 0;
        if navMemory.returningFlag == 0
            navMemory.deployUAV = 1;
            navMemory.returningFlag = navMemory.returningFlag + 1;
        end
        if abs(currentMeasure.ownPosition(1)) < 50 || ...
                abs(currentMeasure.ownPosition(2)) < 50
            input(2) = 0;
        end
        
        navMemory.returning = 1;
        navMemory.trackMode = 0;
        navMemory.avoidanceCounter = 0;
        
    case 12 % State 12 - Prevent UAV From Flying Off Of Map When Tracking
        input(1) = 10;
        input(2) = -0.4;
        navMemory.trackMode = 1;
        
        
    case 15 % State 15 - Speed Up UAV not near chain to make it join the end
        input(1) = 20;
        input(2) = (5*navMemory.dp(kk) + 0.6*navMemory.error(kk));
        navMemory.trackMode = 1;
        navMemory.avoidanceCounter = 0;

end

%------------------------------------------------------------------
% DETERMINE MESSAGE
%------------------------------------------------------------------
% 0 = Not Active, 1 = Track Mode, 2 = Return Flag Turned On, 3 = Newly
% Active Agent, 4 = Otherwise
if navMemory.Active == 0;
    navMemory.logicalMessage = 0;
elseif navMemory.trackMode == 1
    navMemory.logicalMessage = 1;
elseif navMemory.deployUAV == 1
    navMemory.logicalMessage = 2;
elseif navMemory.newlyActive == 1
    navMemory.logicalMessage = 3;
elseif navMemory.parked == 1
    navMemory.logicalMessage = 4;
else
    navMemory.logicalMessage = 5;
end

%------------------------------------------------------------------
% TRANSMIT POSITIONS
%------------------------------------------------------------------
txMsg = [currentMeasure.ownPosition;currentMeasure.p;navMemory.logicalMessage];

%------------------------------------------------------------------
% UPDATE NAV MEMORY
%------------------------------------------------------------------
if input(1) > 0
    navMemory.flightTime = navMemory.flightTime + dt;
end
navMemory.p = currentMeasure.p;
navMemory.lastPos = currentMeasure.ownPosition;

%------------------------------------------------------------------
% LIMIT INPUTS
%------------------------------------------------------------------
if navMemory.navState ~= 1
    if input(2) > 6
        input(2) = 6;
    end
    if input(2) < -6
        input(2) = -6;
    end
end

navMemory.personalCollision = 0;
if navMemory.navState ~= 1;
    if input(1) < 10
        input(1) = 10;
    end
    if input(1) > 20
        input(1) = 20;
    end
    if norm(currentMeasure.ownPosition - currentMeasure.nearestObs) < 50,
        navMemory.personalCollision = 1;
    end
end