%% A short description of the function of your code (a few sentences).
% 
% Assumptions: <e.g. no flow separation, isotropic material>
%   - Assumption1 and short description
% 
% Source: <paper or book (with page number)>
% 
% Inputs: <e.g. variables, structs, files>
%    - input1:
%    - input2:
%
% Outputs: <e.g. variables, structs, files>
%    - output1:
%    - output2:
% 
% Author: Cindi Feng, 2022-02
% ---------------------------------------------------------------------
clear; clc;
%% Design Nonlinear Model Predictive Controller
%% Need to figure out: 
% how to discretize states?
% jacobian state function and constraint function?
% slack variables in workspace limit, collision-free constraint?
% CollisionFreeIneqConFcn: how to implement bounding sphere with radius rc?
%       Why is Bo > rc? 
% cable critical point distance to obstacle 

%% Create NMPC object 
%
% The states are as follows:
%
% x(1:3)   -  xq           inertial frame NED [x,y,z] position [m]
% x(4:6)   -  vq           quad body frame [u,v,w] velocity [m/s]
% x(7:9)   -  eulerAng     euler angles [phi,theta,psi] [rad]
% x(10:12) -  wAng         quad angular rates [p,q,r] [rad/s]
% x(13:14) -  rp           payload frame [rpx,rpy] position [m/s]
% x(15:16) -  vp           payload frame [vpx,vpy] velocity [m/s]
%
% We assume perfect observability, so the output y(k) = x(k).
%
% Control inputs are motor speeds

nx = 16;   
ny = 16; 
nu = 4;     
nlobj = nlmpc(nx,ny,nu);

Ts = 0.1;   % sample time [s]
p = 10;     % prediction horizon
m = 2;      % control horizon

nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = p; % perform trajectory planning 

nlobj.Model.StateFcn = "StateFcn";

%% Parameters for the system
nlobj.Model.NumberOfParameters = 3;

% Parameters to be passed
obstacle = struct;
lim = struct;
params = struct; 

obstacle.detectionDist = 3; 

% Obstacle cuboid dimensions 
obstacle.L = 0.5; 
obstacle.W = 0.5;
obstacle.H = 0.5;

obstacle.xVel = 0.5; % obstacle velocity [m/s]

% Initial obstacle position(s)
obstacle.x = 2;
obstacle.y = 0;
obstacle.z = 2;
obstacle.detectedXo = zeros(3,1);

% Buffer distances
obstacle.Bo = 0.05; % bounding ellipsoid for which collisionss are checked
obstacle.Be = 0.2; % expanded ellipsoid identified as high risk zone in planning

% Workspace limits
lim.Wmin = [-10;-10;0]; % workspace limits
lim.Wmax = [10;10;20];

% Collision-free constraint
params.rc = 0.05; % quad and payload bounding sphere radius

% Quadrotor system parameters
params.cableL = 1.5; % cable length [m]

% Costs parameters
params.qGoal = [3;0;5]; % quad goal position
params.qStart = [0;0;5]; % quad start position

nloptions = nlmpcmoveopt;
nloptions.Parameters = {obstacle,lim,params};

% Initial conditions
x0 = zeros(nx,1);
x0(4:6) = 0.5;
u0 = zeros(nu,1);

%% Constraints
% Input limits
nlobj.MV = struct('Min',{0;0;0;0},'Max',{12;12;12;12}); % limit control inputs

%% Weights
% % Since there are only four manipulated variables, there are not enough 
% % degrees of freedom to track the desired trajectories for all sixteen 
% % output variables (OVs).
% ovweights = zeros(1,16);
% 
% % [x,y,z,phi,theta,psi] are required to follow given ref. trajectory
% ovweights(1:3) = 1;
% ovweights(7:9) = 1;
% 
% nlobj.Weights.OutputVariables = ovweights;
% 
% % MVs also have nominal targets to keep the quad floating
% % set avg MV tracking priority lower than avg OV tracking priority
% nlobj.Weights.ManipulatedVariables = 0.1*ones(1,4);
% 
% % Penalize aggressive control actions by specifying tuning weights for MV
% % rates of change
% nlobj.Weights.ManipulatedVariablesRate = 0.1*ones(1,4);

%% Cost Function
nlobj.Optimization.CustomCostFcn = "CostFcn";
% nlobj.Optimization.ReplaceStandardCost = true;
% TODO: include jacobian

%% Constraint Functions 
% nlobj.Optimization.CustomIneqConFcn = "IneqConFcn_collisionFree";
% nlobj.Optimization.CustomIneqConFcn = "IneqConFcn_workspace";

%% Validation
% double check that everything above looks OK
%validateFcns(nlobj,x0,u0,[],{obstacle,lim,params});

%% Trajectory Planning with Static Obstacle
obstacle.detectedXo = [2;0;2];
nlobj.Optimization.CustomIneqConFcn = "IneqConFcn_collisionFree";
terminal = zeros(1,nx);
terminal(1:3) = params.qGoal;
[~,~,info] = nlmpcmove(nlobj,x0,u0,terminal,[],nloptions);

% Plot quadrotor path
figure;
plot(info.Xopt(:,1),info.Xopt(:,2),'bo')
hold on;
xlabel('x position')
ylabel('y position')
axis equal

% Plot obstacle position
rectangle('Position',[obstacle.x-obstacle.W/2,obstacle.y-obstacle.L/2,...
           obstacle.W,obstacle.L], 'FaceColor',[0.9290 0.6940 0.1250])

%% Planning Simulation 
% 
% T = 0:Ts:4; % simulation time
% xHistory = x0';
% lastMV = zeros(nu,1);
% uHistory = lastMV.';
% 
% for k = 1:length(T)
%     xk = xHistory.';
%     
%     % Switching constraints if obstacle is detected
%     detection = obstacleDetect(xk,obstacle,params.cableL);
%     
%     if sum(detection) > 0
%         obstacle.detectedXo = zeros(3,sum(detection));
%         
%         % Populate detected obstacle positions
%         for i = 1:max(size(detection))
%             if detection(i)
%                 obstacle.detectedXo(:,i) = [obstacle.x(i);obstacle.y(i);obstacle.z(i)];                
%             end
%         end        
%         nlobj.Optimization.CustomIneqConFcn = "IneqConFcn_collisionFree";
%     else
%         nlobj.Optimization.CustomIneqConFcn = "IneqConFcn_workspace";
%     end
%     
%     % Trajectory planning; compute optimal control action for current time
%     [uk,nloptions,info] = nlmpcmove(nlobj,xk,lastMV,[],[],nloptions);
%     uHistory(k+1,:) = uk.';
%     lastMV = uk;
%     
%     % Move obstacle
%     obstacle.x = obstacle.xVel*k*Ts + obstacle.x;
%     obstacle.y = 0*k*Ts + obstacle.y;
%     obstacle.z = 0*k*Ts + obstacle.z;
%     
%     % Plot planned trajectory
%     plot(info.Xopt(:,1),info.Xopt(:,2),'bo')
%     hold on;
%     xlabel('x position')
%     ylabel('y position')
%     axis equal
%     
%     % Plot obstacle position
%     rectangle('Position',[obstacle.x-obstacle.W/2,obstacle.y-obstacle.L/2,...
%                obstacle.W,obstacle.L], 'FaceColor',[0.9290 0.6940 0.1250])
%     
%     % Wait a sec so we can see a pretty animation
%     pause(0.1);
% end  

%% Custom Equations for costs and constraints   
% % Constraints       
% nlobj.Optimization.CustomIneqConFcn = "ConFcn_collisionFreeIneq";
% 
% % Costs
% nlobj.Optimization.CustomCostFcn = "CostFcn";
% nlobj.Optimization.ReplaceStandardCost = true;
% 
% %% Use NMPC and move obstacle
% % Compute optimal control action for current time
% mv = nlmpcmove(nlobj,x,lastmv);