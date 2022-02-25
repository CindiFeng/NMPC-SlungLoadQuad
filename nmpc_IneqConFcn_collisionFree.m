%% Collision Free Inequality Constraint function:
%   Functionality    : Compute custom planning constraints and setup limits
%   Inputs           : X                    - states
%                      U                    - input
%                      e                    - slack
%                      data                 - additional signals
%                      obstacle,lim,params  - optional parameters
%   Outputs          : cineq - inequality constraints; column vector

function cineq = nmpc_IneqConFcn_collisionFree(X,U,e,data,obstacle,lim,params)
p              = data.PredictionHorizon;

Xq             = [X(2:p+1,1).';X(2:p+1,2).';X(2:p+1,3).']; % quad position over p horizon
rp             = [X(2:p+1,7).';X(2:p+1,8).'];
Lz             = sqrt(params.cableL^2-rp.'*rp);
Lz             = diag(Lz).';
Lvec           = [rp;-Lz]; % load pos. w.r.t quad pos.
Xp             = Xq + Lvec; % load position over p horizon

nObs           = obstacle.nXo;
Xo             = obstacle.Xo;
distQuad2Obs   = zeros(p, nObs);
distLoad2Obs   = zeros(p, nObs);
distCable2Obs  = zeros(p, nObs);

for iObs = 1:nObs
    % Point to ellipsoid distance equation (Uteshev and Goncharova 2018)
    [sigma,~] = nmpc_obstacleEllipsoid(obstacle);
    idx1 = 3*iObs - 2;
    idx2 = 3*iObs;
    distQ = Xq - Xo(idx1:idx2,:);
    distL = Xp - Xo(idx1:idx2,:);
    distQuad2Obs(:,iObs) = diag(distQ.' * sigma * distQ)-1;    
    distLoad2Obs(:,iObs) = diag(distL.' * sigma * distL)-1;
    % distCable2Obs(i,:) = 0; 
end

% Put together inequality constraints (<= 0)
obs_coll = zeros(nObs*p,1);

idx = 0;
for dist = [distQuad2Obs,distLoad2Obs]
% for dist = distQuad2Obs
    idx = idx + 1;
    start_idx = p*(idx-1)+1;
    end_idx = p*idx;
    obs_coll(start_idx:end_idx) = dist;
end

cineq = [-obs_coll;
         (Xp(1,:)-lim.Wmax(1)).';
         (Xp(2,:)-lim.Wmax(2)).';
         (Xp(3,:)-lim.Wmax(3)).';
         (-Xp(1,:)+lim.Wmin(1)).';
         (-Xp(2,:)+lim.Wmin(2)).';
         (-Xp(3,:)+lim.Wmin(3)).';
         (Xq(1,:)-lim.Wmax(1)).';
         (Xq(2,:)-lim.Wmax(2)).';
         (Xq(3,:)-lim.Wmax(3)).';
         (-Xq(1,:)+lim.Wmin(1)).';
         (-Xq(2,:)+lim.Wmin(2)).';
         (-Xq(3,:)+lim.Wmin(3)).'];

% cineq = [(Xp(1,:)-lim.Wmax(1)).';
%          (Xp(2,:)-lim.Wmax(2)).';
%          (Xp(3,:)-lim.Wmax(3)).';
%          (-Xp(1,:)+lim.Wmin(1)).';
%          (-Xp(2,:)+lim.Wmin(2)).';
%          (-Xp(3,:)+lim.Wmin(3)).';
%          (Xq(1,:)-lim.Wmax(1)).';
%          (Xq(2,:)-lim.Wmax(2)).';
%          (Xq(3,:)-lim.Wmax(3)).';
%          (-Xq(1,:)+lim.Wmin(1)).';
%          (-Xq(2,:)+lim.Wmin(2)).';
%          (-Xq(3,:)+lim.Wmin(3)).'];

% cineq = [-obs_coll;
%          (Xq(1,:)-lim.Wmax(1)).';
%          (Xq(2,:)-lim.Wmax(2)).';
%          (Xq(3,:)-lim.Wmax(3)).';
%          (-Xq(1,:)+lim.Wmin(1)).';
%          (-Xq(2,:)+lim.Wmin(2)).';
%          (-Xq(3,:)+lim.Wmin(3)).'];
end