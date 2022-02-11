function SlungQuadDynamics(block)
%   Template Copyright 2003-2018 The MathWorks, Inc.
%
%   Functionality:    develop dynamic model of a slung-load quadrotor 
%                     derived from Newton-Euler equations
%
%   Input:            rotor speeds {sigma1...sigma4} [rad/s]
%   
%   Output: 
%   xq               inertial frame NED [x,y,z] position [m]
%   vq               quad body frame [u,v,w] velocity [m/s]
%   eulerAng         euler angles [phi,theta,psi] [rad]
%   wAng             quad angular rates [p,q,r] [rad/s]
%   rp               payload frame [rpx,rpy] position [m/s]
%   vp               payload frame [vpx,vpy] velocity [m/s]
%
%   Author: Cindi Feng 
%
% The setup method is used to set up the basic attributes of the
% S-function such as ports, parameters, etc. Do not add any other
% calls to the main body of the function.
%
setup(block);

end

%% Function: setup ===================================================
% Abstract:
%   Set up the basic characteristics of the S-function block such as:
%   - Input ports
%   - Output ports
%   - Dialog parameters
%   - Options
%
%   Required         : Yes
%   C MEX counterpart: mdlInitializeSizes
%
function setup(block)

% Register number of ports
block.NumInputPorts  = 4;
block.NumOutputPorts = 16;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
for i = 1:block.NumInputPorts
    block.InputPort(i).Dimensions        = 1;
    block.InputPort(i).DatatypeID  = 0;  % double
    block.InputPort(i).Complexity  = 'Real';
    block.InputPort(i).DirectFeedthrough = false;
end

% Override output port properties
for i = 1:block.NumOutputPorts
    block.OutputPort(i).Dimensions       = 1;
    block.OutputPort(i).DatatypeID  = 0; % double
    block.OutputPort(i).Complexity  = 'Real';
end

% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Register number of continuous states
block.NumContStates = 16;

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

% -----------------------------------------------------------------
% The MATLAB S-function uses an internal registry for all
% block methods. You should register all relevant methods
% (optional and required) as illustrated below. You may choose
% any suitable name for the methods and implement these methods
% as local functions within the same file. See comments
% provided for each function for more information.
% -----------------------------------------------------------------

% block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup); % discrete states only
block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
%block.RegBlockMethod('Start', @Start); % Initialize Conditions is used
block.RegBlockMethod('Outputs', @Outputs);     % Required
% block.RegBlockMethod('Update', @Update); % only required for discrete states
block.RegBlockMethod('Derivatives', @Derivatives); % required for continuous states
block.RegBlockMethod('Terminate', @Terminate); % Required

end
%% PostPropagationSetup:
%   Functionality    : Setup work areas and state variables. Can
%                      also register run-time methods here
%   Required         : No
%   C MEX counterpart: mdlSetWorkWidths
%
function DoPostPropSetup(block)
block.NumDworks = 1;
  
  block.Dwork(1).Name            = 'x1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;

end
%% InitializeConditions:
%   Functionality    : Called at the start of simulation and if it is 
%                      present in an enabled subsystem configured to reset 
%                      states, it will be called when the enabled subsystem
%                      restarts execution to reset the states.
%   Required         : No
%   C MEX counterpart: mdlInitializeConditions
%
function InitializeConditions(block)

% % Initialize Dwork
% block.Dwork(1).Data = block.DialogPrm(1).Data;

% Initialize continuous states
block.ContStates.Data(1) = 0; 
block.ContStates.Data(2) = 0;
block.ContStates.Data(3) = 0;
block.ContStates.Data(4) = 0;
block.ContStates.Data(5) = 0;
block.ContStates.Data(6) = 0;
block.ContStates.Data(7) = 0; 
block.ContStates.Data(8) = 0;
block.ContStates.Data(9) = 0;
block.ContStates.Data(10) = 0;
block.ContStates.Data(11) = 0;
block.ContStates.Data(12) = 0;
block.ContStates.Data(13) = 0;
block.ContStates.Data(14) = 0;
block.ContStates.Data(15) = 0;
block.ContStates.Data(16) = 0;

end

%% Start:
%   Functionality    : Called once at start of model execution. If you
%                      have states that should be initialized once, this 
%                      is the place to do it.
%   Required         : No
%   C MEX counterpart: mdlStart
%
function Start(block)

block.Dwork(1).Data = 0;
% block.Dwork(1).Data = [0;0;0];
% block.Dwork(2).Data = [0;0;0];
% block.Dwork(3).Data = [0;0;0];
% block.Dwork(4).Data = [0;0;0];
% block.Dwork(5).Data = [0;0;0];
% block.Dwork(6).Data = [0;0;0];

end

%% Input Port Sampling Method:
function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = 'Sample';
  for i = 1:block.NumOutputPorts
    block.OutputPort(i).SamplingMode  = 'Sample';   
  end
end

%% Outputs:
%   Functionality    : Called to generate block outputs in
%                      simulation step
%   Required         : Yes
%   C MEX counterpart: mdlOutputs

function Outputs(block)

% block.OutputPort(1).Data = block.Dwork(1).Data + block.InputPort(1).Data;

temp_mat = zeros(block.NumContStates,1); 
for i = 1:block.NumContStates
    temp_mat(i) = block.ContStates.Data(i);
end

block.OutputPort(1).Data = temp_mat; % states

end

%% Update:
%   Functionality    : Called to update discrete states
%                      during simulation step
%   Required         : No
%   C MEX counterpart: mdlUpdate
%
function Update(block)

block.Dwork(1).Data = block.InputPort(1).Data;

end

%% Derivatives:
%   Functionality    : Called to update derivatives of
%                      continuous states during simulation step
%   Required         : No
%   C MEX counterpart: mdlDerivatives
%
function Derivatives(block)

% map states and inputs
x        = block.ContStates.Data(1);
y        = block.ContStates.Data(2);
z        = block.ContStates.Data(3);
u        = block.ContStates.Data(4);
v        = block.ContStates.Data(5);
w        = block.ContStates.Data(6);
phi      = block.ContStates.Data(7);
theta    = block.ContStates.Data(8);
psi      = block.ContStates.Data(9);
p        = block.ContStates.Data(10);
q        = block.ContStates.Data(11);
r        = block.ContStates.Data(12);
rpx      = block.ContStates.Data(13);
rpy      = block.ContStates.Data(14);
vpx      = block.ContStates.Data(15);
vpy      = block.ContStates.Data(16);

sig1     = block.InputPort(1).Data(1); 
sig2     = block.InputPort(2).Data(2); 
sig3     = block.InputPort(3).Data(3); 
sig4     = block.InputPort(4).Data(4);

% Parameters
kf = 0.1; % prop thrust coefficient
km = 0.1; % prop torque coefficient
d = 0.3; % quad arm distance [m]
L = 1.5; % payload cable length [m]
mq = 1; % quad mass [kg]
mp = 1; % payload mass [kg]
Ixx = 0.016; % moment of inertia [kg m2]
Iyy = Ixx; % symmetric 
Izz = 0.064; 
Mtot = mp + mq;
g = 9.8; 

% Input force and moment u = [Ff3; taof_vec];
u_vec = [kf kf kf kf; 
       0 d*kf 0 -d*kf;
       -d*kf 0 d*kf 0;
       km km km km]*[sig1;sig2;sig3;sig4];

% Thrust and moment from motor inputs
Ff_vec = [0;0;u_vec(1)];
taof_vec = u_vec(2:4);

% Translational EOM: MVdot + CV = F + G + d
vq = [u;v;w];
rp = [rpx;rpy];
rp2 = rp.'*rp;
g3 = [0;0;g];
vp = [vpx;vpy];

% To describe the kinematic relationship for time derivative of L-vector
B_mat = [1 0;
         0 1;
         -rp.'/sqrt(L^2-rp2)];

Bdot_mat = [zeros(2); 
           -vp.'*L^2/(L^2-rp)^1.5];

G_mat = [mp*(B_mat.')*g3 zeros(2,1); 
         Mtot*g3 zeros(3,1)];

C_mat = [mp*(B_mat.')*Bdot_mat zeros(2,1);
         mp*Bdot_mat zeros(3,1)];

M_mat = [mp*(B_mat.')*B_mat mp*B_mat.';
         mp*B_mat Mtot*eye(3)];

F_mat = [zeros(2,1); Ff_vec];

V_vec = [vp;vq];
Vdot = M_mat\(F_mat+G_mat-C_mat*V_vec);
vpxdot = Vdot(1);
vpydot = Vdot(2);
rpxdot = vpx;
rpydot = vpy;
udot = Vdot(3);
vdot = Vdot(4);
wdot = Vdot(5);

% Rotational EOM: J*wAngdot = taof_vec - wAng_skew*J*wAng
wAng_skew = [0 -r q;
             r 0 -p;
             -q p 0];
J = [Ixx 0 0;
     0 Iyy 0;
     0 0 Izz];

wAngdot = J\(taof_vec - wAng_skew*J*[p;q;r]);
pdot = wAngdot(1);
qdot = wAngdot(2);
rdot = wAngdot(3);

% Find euler angles and inertial frame velocities
eulerAngdot = util_eulerRotMat(phi,theta,p,q,r);
phidot = eulerAngdot(1);
thetadot = eulerAngdot(2);
psidot = eulerAngdot(3);
xqdot = util_rotMat(phi,theta,psi)*vq;
xdot = xqdot(1);
ydot = xqdot(2);
zdot = xqdot(3);

% map derivatives
block.Derivatives.Data(1) = xdot;
block.Derivatives.Data(2) = ydot;
block.Derivatives.Data(3) = zdot;
block.Derivatives.Data(4) = udot;
block.Derivatives.Data(5) = vdot;
block.Derivatives.Data(6) = wdot;
block.Derivatives.Data(7) = phidot;
block.Derivatives.Data(8) = thetadot;
block.Derivatives.Data(9) = psidot;
block.Derivatives.Data(10) = pdot;
block.Derivatives.Data(11) = qdot;
block.Derivatives.Data(12) = rdot;
block.Derivatives.Data(13) = rpxdot;
block.Derivatives.Data(14) = rpydot;
block.Derivatives.Data(15) = vpxdot;
block.Derivatives.Data(16) = vpydot;
end 

%% Terminate:
%   Functionality    : Called at the end of simulation for cleanup
%   Required         : Yes
%   C MEX counterpart: mdlTerminate
%
function Terminate(block)

end 

