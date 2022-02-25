%% Obstacle geometry function:
%   Functionality    : Specify obstacle geometric info
%   Inputs           : obstacle struct 
%                      Xo - obstacle positions of size (3 x p)
%   Outputs          : Ellipsoid bounding sphere (obstacle and extended)

function [sigma_So,sigma_Se] = ms_obstacleEllipsoid(obstacle)

% obstacle position fixed to center of cuboid
% obstacle cuboid edges (le,we,he)
le = obstacle.L; 
we = obstacle.W;
he = obstacle.H;

% create obstacle ellipsoid using constraint maximization
% semi-principal axes (a,b,c)
a = sqrt(3)/2*le; 
b = sqrt(3)/2*we;
c = sqrt(3)/2*he;

% Generate safe zones
SoVec = [a+obstacle.Bo;b+obstacle.Bo;c+obstacle.Bo];
SeVec = [a+obstacle.Be;b+obstacle.Be;c+obstacle.Be];

sigma_So = diag([1/SoVec(1)^2,1/SoVec(2)^2,1/SoVec(3)^2]);
sigma_Se = diag([1/SeVec(1)^2,1/SeVec(2)^2,1/SeVec(3)^2]);

end