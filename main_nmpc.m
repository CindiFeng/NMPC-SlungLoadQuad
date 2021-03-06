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

nx = 10;   
ny = 10; % c;w_x;w_y;w_z
nu = 4;     
nlobj = nlmpc(nx,ny,nu);

Ts = 0.1;   % sample time [s]
p = 60;     % prediction horizon
m = 2;      % control horizon

nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = p; % perform trajectory planning 
% nlobj.ControlHorizon = m; % tracking  

nlobj.Model.StateFcn = "QuadrotorUZHStateFcn";

%% Parameters for the system
nlobj.Model.NumberOfParameters = 3;

% Parameters to be passed
obstacle = nmpc_obstacleTraj(p);
lim = struct;
params = struct; 

% Workspace limits
lim.Wmin = [-10;-10;0]; % workspace limits
lim.Wmax = [10;10;20];

% Quadrotor system parameters
params.cableL = 1.5; % cable length [m]

% Costs parameters
params.qGoal = [3;0;5]; % quad goal position
params.qStart = [-1;0;0]; % quad start position

nloptions = nlmpcmoveopt;
nloptions.Parameters = {obstacle,lim,params};

% Initial conditions
x0 = zeros(nx,1);
x0(1:3) = params.qStart;
x0(4) = 1;
% x0(4:6) = 0.5;
u0 = zeros(nu,1);
u0(1) = 6;

%% Constraints
% Input limits
% nlobj.MV = struct('Min',{0;0;0;0},'Max',{12;12;12;12}); % limit control inputs
fmin = 0.5; fmax = 5;
Tmin = 4*fmin; Tmax = 4*fmax;
w_max_yaw = 1;     % Maximal yaw rate [rad/s]
w_max_xy = 3;      % Maximal pitch and roll rate [rad/s]

ConsMin = {Tmin; -w_max_xy; -w_max_xy; -w_max_yaw};
ConsMax = {Tmax; w_max_xy; w_max_xy; w_max_yaw};
nlobj.MV = struct('Min',ConsMin,'Max',ConsMax);

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
nlobj.Optimization.CustomCostFcn = "nmpc_CostFcn";
nlobj.Optimization.ReplaceStandardCost = true;
% TODO: include jacobian

%% Constraint Functions 
nlobj.Optimization.CustomIneqConFcn = "nmpc_IneqConFcn_collisionFree";
% nlobj.Optimization.CustomIneqConFcn = "nmpc_IneqConFcn_workspace";

%% Validation
% double check that everything above looks OK
validateFcns(nlobj,x0,u0,[],{obstacle,lim,params});

%% Trajectory Planning with Static Obstacle
Xgoal = [2,0,3,1,0,0,0,0,0,0];
Xgoal(1:3) = params.qGoal;
Xref = repmat(Xgoal,length(p),1);
[~,~,info] = nlmpcmove(nlobj,x0,u0,Xref,[],nloptions);

% Plot quadrotor path
figure;
plot(info.Xopt(:,1),info.Xopt(:,2),'bo')
hold on;
xlabel('x position')
ylabel('y position')
axis equal

% Plot obstacle position
rectangle('Position',[obstacle.Xo(1)-obstacle.W/2,obstacle.Xo(2)-obstacle.L/2,...
           obstacle.W,obstacle.L], 'FaceColor',[0.9290 0.6940 0.1250])

%% Tracking Simulation 

Duration = 4;
T = 0:Ts:Duration; % simulation time steps
xHistory = x0';
lastMV = zeros(nu,1);
uHistory = lastMV';

% Custom reference trajectory
Xref = nmpc_referenceTrajectory(params,length(T),p);
Xgoal = [2,0,3,1,0,0,0,0,0,0];
Xgoal(1:3) = params.qGoal;
Xref = repmat(Xgoal,length(T),1);

figure;
plot(Xref(:,1), Xref(:,2),'bo')
hold on
xlabel('x position')
ylabel('y position')

% Plot obstacle position
rectangle('Position',[obstacle.Xo(1)-obstacle.W/2,obstacle.Xo(2)-obstacle.L/2,...
           obstacle.W,obstacle.L], 'FaceColor',[0.9290 0.6940 0.1250])
axis equal

hbar = waitbar(0,'Simulation Progress');
for k = 1:length(T)
    plot(xHistory(k,1),xHistory(k,2),'rx')
    xk = xHistory(k,:);
    
    % Trajectory planning; compute optimal control action for current time
    [uk,nloptions,info] = nlmpcmove(nlobj,xk,lastMV,Xref(k:min(k+p-1,length(T)),:),[],nloptions);
    uHistory(k+1,:) = uk';
    lastMV = uk;
    
    % Update states.
%     ODEFUN = @(t,xk) nmpc_StateFcn(xk,uk,obstacle,lim,params);
    ODEFUN = @(t,xk) QuadrotorUZHStateFcn(xk,uk,obstacle,lim,params);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    xHistory(k+1,:) = YOUT(end,:);
    waitbar(k*Ts/Duration,hbar);
    
    % plot planned trajectory
    x1_plan = info.Xopt(:,1);
    x2_plan = info.Xopt(:,2);
    plot(x1_plan,x2_plan,'g-');
    
%     % Wait a sec so we can see a pretty animation
%     pause(0.1);
end  
close(hbar)

legend('Planned Trajectory','Actual Trajectory','NMPC Plan','Location','northeast')

%% Custom Equations for costs and constraints   
% % Constraints       
% nlobj.Optimization.CustomIneqConFcn = "track_IneqConFcn_collisionFree";
% 
% % Costs
% nlobj.Optimization.CustomCostFcn = "track_CostFcn";
% nlobj.Optimization.ReplaceStandardCost = true;
% 
% %% Use NMPC and move obstacle
% % Compute optimal control action for current time
% mv = nlmpcmove(nlobj,x,lastmv);