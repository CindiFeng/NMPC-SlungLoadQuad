%% Obstacle geometry function:
%   Functionality    : Specify obstacle geometric info
%   Inputs           : obstacle struct and obstacle positions
%   Outputs          : Ellipsoid bounding sphere (obstacle and extended)

function [SoVec,SeVec] = obstacleGeometry(obstacle)

% obstacle position fixed to center of cuboid
% obstacle cuboid edges (le,we,he)
le = obstacle.L/2; 
we = obstacle.W/2;
he = obstacle.H/2;

% create obstacle ellipsoid using constraint maximization
% semi-principal axes (a,b,c) = (x,y,z)
a = sqrt(3)/2*le; 
b = sqrt(3)/2*we;
c = sqrt(3)/2*he;

% Generate safe zones
SoVec = [a+obstacle.Bo;b+obstacle.Bo;c+obstacle.Bo];
SeVec = [a+obstacle.Be;b+obstacle.Be;c+obstacle.Be];

end