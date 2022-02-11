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
nu = 4;     
Ts = 0.1;   % sample time [s]
p = 26;     % prediction horizon

nlobj = nlmpcMultistage(p,nx,nu);

nlobj.Ts = Ts;

nlobj.Model.StateFcn = "ms_StateFcn";

%% Parameters for the system
% Parameters to be passed

rc = 0.05; % quad and payload bounding sphere radius
cableL = 1.5; % cable length [m]
params = [rc; cableL];

% Initial conditions
x0 = zeros(nx,1);
x0(4:6) = 0.5;
u0 = zeros(nu,1);

qGoal = [6;0;5]; % quad goal position
qStart = [0;0;5]; % quad start position

%% Constraints
% hard bounds on the mainupulated variables (input)
nlobj.MV = struct('Min',{0;0;0;0},'Max',{12;12;12;12}); % limit control inputs

%% Custom Cost and Constraint Function
% TODO: include jacobian
for ct = 1:p
    if ct == p
        nlobj.Stages(ct).CostFcn =  "ms_CostFcn_terminal";
    else 
        nlobj.Stages(ct).CostFcn =  "ms_CostFcn_stage";
    end
    nlobj.Stages(ct).ParameterLength = 2;
end

%% Constraint Functions 
for ct = 2:p
    nlobj.Stages(ct).IneqConFcn = "ms_IneqConFcn";
    nlobj.Stages(ct).ParameterLength = 2;
end

%% 
terminalPos = zeros(nx,1);
terminalPos(1:3) = qGoal;
nlobj.Model.TerminalState = terminalPos;

%% Validation
% double check that everything above looks OK
simdata = getSimulationData(nlobj,'TerminalState');
simdata.StageParameter = repmat([rc; cableL],p,1);
simdata.TerminalState = terminalPos;
validateFcns(nlobj,x0,u0,simdata)

%% Trajectory Planning with Static Obstacle
nlobj.Optimization.SolverOptions.MaxIterations = 1000;

[~,~,info] = nlmpcmove(nlobj,x0,u0,simdata);

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