function dxdt = MS_StateFcn(x_state,input)
% Functionality:    develop dynamic model of a slung-load quadrotor 
%                     derived from Newton-Euler equations
%
% Input:            u(1:4) rotor speeds {sigma1...sigma4} [rad/s]
%   
% Output: 
% x(1:3)   -  xq           inertial frame NED [x,y,z] position [m]
% x(4:6)   -  vq           quad body frame [u,v,w] velocity [m/s]
% x(7:9)   -  eulerAng     euler angles [phi,theta,psi] [rad]
% x(10:12) -  wAng         quad angular rates [p,q,r] [rad/s]
% x(13:14) -  rp           payload frame [rpx,rpy] position [m/s]
% x(15:16) -  vp           payload frame [vpx,vpy] velocity [m/s]

%% SPECIFY: 
% Obstacle model 
obstacle = struct;
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

%% Variables
dxdt = zeros(16,1);
x        = x_state(1);
y        = x_state(2);
z        = x_state(3);
u        = x_state(4);
v        = x_state(5);
w        = x_state(6);
phi      = x_state(7);
theta    = x_state(8);
psi      = x_state(9);
p        = x_state(10);
q        = x_state(11);
r        = x_state(12);
rpx      = x_state(13);
rpy      = x_state(14);
vpx      = x_state(15);
vpy      = x_state(16);

sig1     = input(1); 
sig2     = input(2); 
sig3     = input(3); 
sig4     = input(4);

% Input force and moment u = [Ff3; taof_vec];
u_vec = [   kf     kf      kf      kf; 
            0    d*kf       0   -d*kf;
         -d*kf      0    d*kf       0;
            km     km      km      km] * [sig1;sig2;sig3;sig4];

% Thrust and moment from motor inputs
Ff_vec = [0; 0; u_vec(1)];
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
           -vp.'*L^2/(L^2-rp2).^1.5];

G_mat = [mp*(B_mat.')*g3 zeros(2,1); 
         Mtot*g3 zeros(3,1)];

C_mat = [mp*(B_mat.')*Bdot_mat zeros(2,3);
         mp*Bdot_mat zeros(3)];

M_mat = [mp*(B_mat.')*B_mat mp*B_mat.';
         mp*B_mat Mtot*eye(3)];

F_mat = [zeros(2,1); Ff_vec];

V_vec = [vp;vq];
Vdot = M_mat\(F_mat+G_mat-C_mat*V_vec);
dxdt(15) = Vdot(1);
dxdt(16) = Vdot(2);
dxdt(13) = vpx;
dxdt(14) = vpy;
dxdt(4) = Vdot(3);
dxdt(5) = Vdot(4);
dxdt(6) = Vdot(5);

% Rotational EOM: J*wAngdot = taof_vec - wAng_skew*J*wAng
wAng_skew = [ 0 -r  q;
              r  0 -p;
             -q  p  0];
J = [Ixx   0   0;
       0 Iyy   0;
       0   0 Izz];

wAngdot = J\(taof_vec - wAng_skew*J*[p;q;r]);
dxdt(10) = wAngdot(1);
dxdt(11) = wAngdot(2);
dxdt(12) = wAngdot(3);

% Find euler angles and inertial frame velocities
eulerAngdot = util_eulerRotMat(phi,theta,p,q,r);
dxdt(7) = eulerAngdot(1);
dxdt(8) = eulerAngdot(2);
dxdt(9) = eulerAngdot(3);
xqdot = util_rotMat(phi,theta,psi)*vq;
dxdt(1) = xqdot(1);
dxdt(2) = xqdot(2);
dxdt(3) = xqdot(3);