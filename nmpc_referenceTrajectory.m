function [xdesired] = nmpc_referenceTrajectory(params,Tsteps,p)
start = params.qStart;
goal = params.qGoal;
% xSpeed = (goal(1) - start(1))/p;

x        = goal(1)*ones(Tsteps,1); x(1:p) = linspace(start(1),goal(1),p);
y        = zeros(Tsteps,1);
z        = (goal(3) - start(3))*ones(Tsteps,1);
% u        = zeros(Tsteps,1); u(1:p) = xSpeed;
% v        = zeros(Tsteps,1);
% w        = zeros(Tsteps,1);
% phi      = zeros(Tsteps,1);
% theta    = zeros(Tsteps,1);
% psi      = zeros(Tsteps,1);
% p        = zeros(Tsteps,1);
% q        = zeros(Tsteps,1);
% r        = zeros(Tsteps,1);
% rpx      = x;
% rpy      = zeros(Tsteps,1);
% vpx      = zeros(Tsteps,1); vpx(1:p) = xSpeed;
% vpy      = zeros(Tsteps,1);
% 
% xdesired = [x,y,z,u,v,w,phi,theta,psi,p,q,r,rpx,rpy,vpx,vpy];

q_w = ones(Tsteps,1);
q_x = zeros(Tsteps,1);
q_y = zeros(Tsteps,1);
q_z = zeros(Tsteps,1);
v_x = zeros(Tsteps,1);
v_y = zeros(Tsteps,1);
v_z = zeros(Tsteps,1);

xdesired = [x,y,z,q_w,q_x,q_y,q_z,v_x,v_y,v_z];
end