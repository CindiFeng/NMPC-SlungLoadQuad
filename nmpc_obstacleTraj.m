%% Obstacle Detection function:
%   Functionality    : Specify obstacle parameters and trajectories
%                      
%   Inputs           : p - nmpc prediction horizon
%   Outputs          : obstacle struct

function obstacle = nmpc_obstacleTraj(p,k,Ts)
% Collision-free constraint
rc = 0.1; % quad and payload bounding sphere radius

obstacle = struct;

% Obstacle cuboid dimensions 
obstacle.L = 0.25; 
obstacle.W = 0.25;
obstacle.H = 6;

% Buffer distances
obstacle.Bo = rc + 0.01; % bounding ellipsoid for which collisionss are checked
obstacle.Be = 0.1; % expanded ellipsoid identified as high risk zone in planning

% Obstacle waypoints 
obsVel = [0;0;0];
obsAcc = [0;0;0];
Xo0 = [2;0;3];
Xo0 = Xo0 + obsVel * k*Ts + 0.5 * obsAcc * (k*Ts)^2; % Initial obstacle position(s) [x1;y1;z1;x2...znXo]
obstacle.Xo = zeros(max(size(Xo0)),p);
obstacle.nXo = max(size(Xo0)) / 3; % number of obstacles
for i = 1:p
    % x = x0 + v0*t + 0.5*a*t^2
    obstacle.Xo(:,i) = Xo0 + obsVel * i + 0.5 * obsAcc * i^2;   
end

end