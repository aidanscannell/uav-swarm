function [navMemory] = setup(Nagents, Active_Agents,aa)
%------------------------------------------------------------------
%SETUP AGENT MEMORY
%------------------------------------------------------------------
navMemory.personalCollision = 0;
navMemory.Collisions = 0;
navMemory.cloudPerimeter = 0;

%------------------------------------------------------------------
% INITIALISE MEMORY
%------------------------------------------------------------------
% Initialise Recorded Position
navMemory.lastPos = [0;0];

% Initialise Operating State
navMemory.navState = 1;

% Initialise PPM Store
navMemory.p = 0;

% Initialise Flight Time
navMemory.flightTime = 0;

% Initialise Agent Number
navMemory.number = aa;

% Initialise Heading Estimate
navMemory.headEstimate =0;

% Initialise Target In Cloud For Agents To Fly Towards
navMemory.swarmPosition = [0;0];

% Initialise Concentration Map
navMemory.concMap = zeros(2000); %MAKE uint32
%navMemory.concMap = zeros(2000, 'uint32');

% Initialise Controller For Tracking Cloud
navMemory.dp = 0;
navMemory.error = 0;

% Initialise Turning Controller
navMemory.turningError = 0;
navMemory.returningError = 0;

% Initialise Message Dependant on Agents State
navMemory.logicalMessage = 0;

% Initialise Number of Active Agents
navMemory.Active_Agents = Active_Agents;

% Initialise Counter For Starting UAVs During Simulation
navMemory.initialCounter = 0;


%------------------------------------------------------------------
% COUNTERS
%------------------------------------------------------------------
% Counter To Spread Initial Agents
navMemory.spreadCounter = 0;

% Counter For Avoidance 
navMemory.avoidanceCounter = 0;

% Counter For Number of Agents In Cloud
navMemory.uav_counter = 0;

% Counter For Number of Collisions
navMemory.collision = 0;


%------------------------------------------------------------------
% FLAGS
%------------------------------------------------------------------
% Flag For Active UAVs (1 == Active, 0 == In Active)
navMemory.Active = 0;

% Flag For Newly Active Agents
navMemory.newlyActive = 0;

% Flag For Returning Agents (1 == Agent is returning)
navMemory.returning = 0;

% Flag For Returning UAVs (1 == Just Started Returning, 0 == Otherwise)
navMemory.returningFlag = 0;

% Flag To Deploy UAV
navMemory.deployUAV = 0;

% Flags For Spacing Agents In Cloud
navMemory.speedUpFlag = 0;
navMemory.slowDownFlag = 0;

% Flag For Closest Robot To Fly To Cloud
navMemory.flyingToCloud = 0;

% Flag For Parked Agents
navMemory.parked = 0;

% Flag For Agents Tracking Cloud
navMemory.trackMode = 0;

