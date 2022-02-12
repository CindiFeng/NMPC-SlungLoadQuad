function dxdt = QuadrotorUZHStateFcn(x,u,~,~,~)
% State equations of 3D VPC

% Parameters
arm_length = 0.17;               % length of rotor arm
mass = 0.73;                     % mass of quadrotor
J = diag([0.007, 0.007, 0.012]); % moment of inertia
J_inv = diag([142.8571, 142.8571, 83.3333]); % inverse of moment of inertia
g_z = 9.81;                  % gravity
R_cb = RotateWtoB(-pi/2,0,-pi/2);
R_bc = R_cb';

% Variables
p_x = x(1);
p_y = x(2);
p_z = x(3);
q_w = x(4);
q_x = x(5);
q_y = x(6);
q_z = x(7);
v_x = x(8);
v_y = x(9);
v_z = x(10);

c = u(1)/mass;
w_x = u(2);
w_y = u(3);
w_z = u(4);

% State equations
dxdt = zeros(10,1);
dxdt(1) = v_x;
dxdt(2) = v_y;
dxdt(3) = v_z;

dxdt(4) = 0.5 * (- w_x * q_x - w_y * q_y - w_z * q_z); % dq_w
dxdt(5) = 0.5 * (w_x * q_w + w_z * q_y - w_y * q_z);   % dq_x
dxdt(6) = 0.5 * (w_y * q_w - w_z * q_x + w_x * q_z);   % dq_y
dxdt(7) = 0.5 * (w_z * q_w + w_y * q_x - w_x * q_y);   % dq_z

dxdt(8) = 2 * (q_w * q_y + q_x * q_z) * c;
dxdt(9) = 2 * (q_y * q_z - q_w * q_x) * c;
dxdt(10) = (1 - 2 * q_x * q_x - 2 * q_y * q_y) * c - g_z;



