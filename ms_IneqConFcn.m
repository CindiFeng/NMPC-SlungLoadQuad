%% Collision Free Inequality Constraint function:
%   Functionality    : Compute custom planning constraints and setup limits
%   Inputs           : X                    - states
%                      U                    - input
%                      e                    - slack
%                      data                 - additional signals
%                      obstacle,lim,params  - optional parameters
%   Outputs          : cineq - column vector

function cineq = ms_IneqConFcn(stage,x,u,p)
%% SPECIFY: 
% Workspace limits
Wmin = [-10;-10;0]; % workspace limits
Wmax = [10;10;20];

%% Determine obstacle distances
obstacle = ms_obstacleTraj(stage);

rc = p(1);
cableL = p(2);

Xq = [x(1);x(2);x(3)]; % quad position
rp = [x(13);x(14)];
Lz = sqrt(cableL^2-rp.'*rp);
Lz = diag(Lz).';
Lvec = [rp;Lz]; % load pos. w.r.t quad pos.
Xp = Xq + Lvec; % load position
Xcp = 0;

nXo           = obstacle.nXo;
Xo            = obstacle.Xo;
distQuad2Obs  = zeros(nXo,1);
distLoad2Obs  = zeros(nXo,1);
distCable2Obs = zeros(nXo,1);

for i = 1:nXo
    % Point to ellipsoid distance equation (Uteshev and Goncharova 2018)
    [sigma,~] = nmpc_obstacleEllipsoid(obstacle);
    idx1 = 3*i - 2;
    idx2 = 3*i;
    distQ = Xq + eye(3,1)*rc - Xo(idx1:idx2,stage);
    distL = Xp + eye(3,1)*rc - Xo(idx1:idx2,stage);
    distQuad2Obs(i) = diag(distQ.' * sigma * distQ)-1;    
    distLoad2Obs(i) = diag(distL.' * sigma * distL)-1;
    % distCable2Obs(i,:) = 0; 
end

% Put together inequality constraints (<= 0)
cineq = [distQuad2Obs;
         distLoad2Obs;
         Xp-Wmax;
         -Xp+Wmin;
         Xq-Wmax;
         -Xq+Wmin];
end