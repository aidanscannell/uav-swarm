function simulation
%------------------------------------------------------------------
% INITIALISE TIME STEP & VARIABLES
%------------------------------------------------------------------
tic
dt = 3.6;
t=0;
activeCounter = 0;
numberOfCollision = 0;
readToDeploy = 0;

%------------------------------------------------------------------
% LOAD CLOUD
%------------------------------------------------------------------
load ('cloud2');
% cloud.x = -cloud.x;
%cloud.p = 0;
%------------------------------------------------------------------
% INPUTS
%------------------------------------------------------------------
Nagents = 6;
% Active_Agents = Nagents/3;
Active_Agents = Nagents;

%------------------------------------------------------------------
% SETUP DYNAMIC STATES, INPUTS & STATE STORE
%------------------------------------------------------------------
x = [3*2*(rand(2,Nagents)-0.5)-3;0:360*Nagents/Active_Agents/(Nagents):360*Nagents/Active_Agents*...
    (Nagents-1)/Nagents];
for aa = 1 : Nagents
    input{aa} = [zeros(2,1)];
    xs = zeros(3,Nagents);
end

%------------------------------------------------------------------
% SETUP AGENT MEMORY
%------------------------------------------------------------------
for aa = 1 : Nagents
    [navMemory{aa}] = setup(Nagents, Active_Agents,aa);
end
for aa = Active_Agents+1 : Nagents
    navMemory{aa}.parked = 1;
end

%------------------------------------------------------------------
% ACTIVATE HALF OF AGENTS
%------------------------------------------------------------------
for aa = 1 : Active_Agents
    navMemory{aa}.Active = 1;
end

%------------------------------------------------------------------
% SETUP CHANNEL SIMULATION FOR COMMS
%------------------------------------------------------------------
channel = initChannel();


%------------------------------------------------------------------
% TIME LOOP
%------------------------------------------------------------------
for kk=1:20000,
    
    t = t + dt;

    for aa = 1 : Nagents,
        
        % SIMULATE OWN POSITION MEMORY
        currentMeasure{aa} = simMeas(x,aa,cloud,t);
        
        % SIMULATE RECEIVED MESSAGE
        [rxMsgs{aa},channel] = simReceive(aa,channel);
        
        number_in_cloud = 0;
        for ii = 1 : Nagents
            if navMemory{ii}.trackMode == 1
                number_in_cloud = number_in_cloud + 1;
            end
        end
        
        % MAKE DECISION ON VELOCITY AND CURVATURE
        [input{aa},navMemory{aa},txMsgs{aa}] = simNavDecision(currentMeasure{aa},navMemory{aa},Active_Agents,aa,rxMsgs{aa},t,dt,kk,number_in_cloud);
        
        % SIMULATE TRANSMISSION
        channel = simTransmit(txMsgs{aa},aa,channel);
        
        % STORE OLD STATES AND INPUTS
        navMemory{aa}.xs_1(:,kk) = x(1,:);
        navMemory{aa}.xs_2(:,kk) = x(2,:);
        navMemory{aa}.us(1:2,kk) = input{aa};
        
    end
    
    %------------------------------------------------------------------
    % SIMULATE THE COMMS
    %------------------------------------------------------------------
    channel = simChannel(channel,x);
    
    %------------------------------------------------------------------
    % CALCULATE ANY COLLISIONS
    %------------------------------------------------------------------
    nearestDist = inf;
    for ii = 1 : Nagents,
        for jj = 1 : Nagents,
            if ii ~= jj && navMemory{ii}.Active == 1 && navMemory{jj}.Active == 1 & kk > 10 &&...
                    navMemory{ii}.parked ~= 1 && navMemory{jj}.parked ~= 1
                dist = norm(x(1:2,ii) - x(1:2,jj));
                if dist < 50
                    numberOfCollision = numberOfCollision + 1/2;
                    integerTest=~mod(numberOfCollision,1);
                    if integerTest == 1
                        disp(['Number of Collisions - ' num2str(numberOfCollision)])
                        pause(0.1)
                    end
                end
            end
        end
    end
    
    %------------------------------------------------------------------
    %PLOT THE UAV AND CLOUD
    %------------------------------------------------------------------ 
    %if t > 900
        plot_simulation(Nagents,Active_Agents,navMemory,x,kk,cloud,t,numberOfCollision)
    %end
    
%     for aa = 1 : Nagents,
%         if navMemory{aa}.trackMode == 1 || navMemory{aa}.p > 1
%             timeToFindCloud = t
%              return
%         end
%     end
%     numberInCloud = 0;
%     for aa = 1 : Nagents,
%         if navMemory{aa}.trackMode == 1 || navMemory{aa}.p > 1
%             numberInCloud = numberInCloud + 1;
%             if numberInCloud == 6
%                 timeForAllAgentsToTrackCloud = t
%                  return
%             end
%         end
%     end
%     for ii = 1 : Nagents
%         perimeter(ii) = navMemory{ii}.cloudPerimeter;
%     end
%     if t >= 1500
%         disp(num2str(numberOfCollision))
%         perimeter
%         toc
%         return
%     end
    
    %------------------------------------------------------------------
    %EXECUTE DECIDED MOVEMENT
    %------------------------------------------------------------------
    for aa = 1 : Nagents,
        x(:,aa) = simMove(x(:,aa),input{aa},dt);
%         x(1,aa) = x(1,aa) + 0.8*randn(1);
%         x(2,aa) = x(2,aa) + 0.8*randn(1);
%         x(3,aa) = x(3,aa) + 0.1*(0.1 + (1-0.1)*rand(1,1));
    end  
     
end

%------------------------------------------------------------------
% TAKE MEASUREMENTS
%------------------------------------------------------------------
function [currentMeasure] = simMeas(x,aa,cloud,t)
currentMeasure.p = cloudsamp(cloud,x(1,aa),x(2,aa),t);
currentMeasure.ownPosition = x(1:2,aa) + 3*(1 + (1-3)*rand(1,1));

%------------------------------------------------------------------
% SIMULATE MOVEMENT
%------------------------------------------------------------------
function xnew=simMove(x,input,dt)
% Runge Kutta 4th Order
k1 = f_continuous(x,input);
k2 = f_continuous(x+k1*dt/2,input);
k3 = f_continuous(x+k2*dt/2,input);
k4 = f_continuous(x+k3*dt,input);
xnew = x+(k1+2*k2+2*k3+k4)*dt/6;

%------------------------------------------------------------------
% STATE SPACE
%------------------------------------------------------------------
function xdot=f_continuous(x,input,dt)
xdot = [input(1)*sind(x(3)); input(1)*cosd(x(3)); input(1)*input(2)];

%------------------------------------------------------------------
% COMMUNICATIONS
%------------------------------------------------------------------
function channel = initChannel()
% initialize comms channel model
channel.curMsgs = {};
channel.newMsgs = {};

function [rxMsgs,channel] = simReceive(aa,channel)
% simulate receiving messages
% simple broadcast model - just get everything
rxMsgs = channel.curMsgs;

function channel = simTransmit(txMsgs,aa,channel)
% simulate transmitting a message
% store it for next step
channel.newMsgs = [channel.newMsgs txMsgs];

function channel = simChannel(channel,x)
% simple - everyone gets everything
channel.curMsgs = channel.newMsgs;
channel.newMsgs = {};