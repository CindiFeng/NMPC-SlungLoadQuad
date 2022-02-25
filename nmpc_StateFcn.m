function dxdt = nmpc_StateFcn(x_state,input,~,~,~)
% Functionality:    develop dynamic model of a slung-load quadrotor 
%                     derived from Newton-Euler equations
%
% Input:            u(1:4) rotor speeds {sigma1...sigma4} [rad/s]
% Input:            forces in body frame
%   
% Output: 
% x(1:3)   -  xq           inertial frame NED [x,y,z] quad position [m]
% x(4:6)   -  vq           inertial frame quad velocity [m/s]
% x(7:8)   -  rp           payload frame [rpx,rpy] position [m]
% x(9:10)  -  vp           payload frame [vpx,vpy] velocity [m/s]
% x(11:13) -  wAng         body frame quad angular rates [p,q,r] [rad/s]

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

% Variables
dxdt     = zeros(10,1);
xq_x     = x_state(1);
xq_y     = x_state(2);
xq_z     = x_state(3);
vq_x     = x_state(4);
vq_y     = x_state(5);
vq_z     = x_state(6);
rp_x     = x_state(7);
rp_y     = x_state(8);
vp_x     = x_state(9);
vp_y     = x_state(10);
% p        = x_state(11);
% q        = x_state(12);
% r        = x_state(13);
% phi      = x_state(14);
% theta    = x_state(15);
% psi      = x_state(16);
% 
% sig1     = input(1); 
% sig2     = input(2); 
% sig3     = input(3); 
% sig4     = input(4);
fx = input(1);
fy = input(2);
fz = input(3);

% Input force and moment u = [Ff3; taof_vec];
% u_vec = [   kf     kf      kf      kf; 
%             0    d*kf       0   -d*kf;
%          -d*kf      0    d*kf       0;
%             km     km      km      km] * [sig1;sig2;sig3;sig4];

% Thrust and moment from motor inputs
% Ff_vec = util_rotMat(phi,theta,psi)*[0; 0; -u_vec(1)];
% taof_vec = u_vec(2:4);
Ff_vec = [fx;fy;fz];

% Translational EOM: MVdot + CV = F + G + d
vq = [vq_x;vq_y;vq_z];
rp = [rp_x;rp_y];
rp2 = rp.'*rp;
g3 = [0;0;g];
vp = [vp_x;vp_y];

% Kinematic relationship for time derivative of L-vector
B_mat = [eye(2);
         -rp.'/sqrt(L^2-rp2)];

Bdot_mat = [zeros(2); 
           (vp*(rp2-L^2)-rp*rp'*vp)'/(L^2-rp2)^1.5];

G_mat = [mp*(B_mat.')*g3; 
         Mtot*g3];

C_mat = [mp*(B_mat.')*Bdot_mat zeros(2,3);
         mp*Bdot_mat zeros(3)];

M_mat = [mp*(B_mat.')*B_mat mp*B_mat.';
         mp*B_mat Mtot*eye(3)];

F_mat = [zeros(2,1); Ff_vec];

V_vec = [vp;vq];

% Translational EOM: MVdot + CV = F + G + d
Vdot = M_mat\(F_mat+G_mat-C_mat*V_vec);
dxdt(9) = Vdot(1);
dxdt(10) = Vdot(2);
dxdt(4) = Vdot(3);
dxdt(5) = Vdot(4);
dxdt(6) = Vdot(5);

dxdt(7) = vp_x;
dxdt(8) = vp_y;
dxdt(1) = vq_x;
dxdt(2) = vq_y;
dxdt(3) = vq_z;

% % Rotational EOM: J*wAngdot = taof_vec - wAng_skew*J*wAng
% wAng_skew = [ 0 -r  q;
%               r  0 -p;
%              -q  p  0];
% J = [Ixx   0   0;
%        0 Iyy   0;
%        0   0 Izz];
% 
% wAngdot = J\(taof_vec - wAng_skew*J*[p;q;r]);
% dxdt(11) = wAngdot(1);
% dxdt(12) = wAngdot(2);
% dxdt(13) = wAngdot(3);
% 
% % Find euler angles and inertial frame velocities
% eulerAngdot = util_eulerRotMat(phi,theta,p,q,r);
% dxdt(14) = eulerAngdot(1);
% dxdt(15) = eulerAngdot(2);
% dxdt(16) = eulerAngdot(3);
% xqdot = util_rotMat(phi,theta,psi)*vq;
% dxdt() = xqdot(1);
% dxdt() = xqdot(2);
% dxdt() = xqdot(3);