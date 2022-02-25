clear; clc;

qStart = [1;0;3]; % quad start position
x0 = zeros(10,1);
x0(1:3) = qStart;
swing_angle = 10/180*pi;
rpx0 = 1.5*sin(swing_angle)*cos(swing_angle);
rpy0 = 1.5*(sin(swing_angle))^2;
x0(7:8) = [rpx0;rpy0];
u0 = 0*ones(3,1);
u0(1) = 2;
u0(2) = -2;
u0(3) = -10;
u0 = [1.12809787661550,-0.0607815743902776,-28.4061575714604]';
u0 = [500;0;200];
ODEFUN = @(t,xk) nmpc_StateFcn(xk,u0,1,1,1);
[TOUT,YOUT] = ode45(ODEFUN,[0 0.1], x0);
xHistory = YOUT(end,:);