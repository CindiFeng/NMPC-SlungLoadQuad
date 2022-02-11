%% Collision Free Inequality Constraint function:
%   Functionality    : Compute custom planning constraints and setup limits
%   Inputs           : X                    - states
%                      U                    - input
%                      e                    - slack
%                      data                 - additional signals
%                      obstacle,lim,params  - optional parameters
%   Outputs          : cineq - column vector

function cineq = nmpc_IneqConFcn_collisionFree(X,U,e,data,obstacle,lim,params)
p             = data.PredictionHorizon;

Xq            = [X(2:p+1,1).';X(2:p+1,2).';X(2:p+1,3).']; % quad position over p horizon
rp            = [X(2:p+1,13).';X(2:p+1,14).'];
Lz            = sqrt(params.cableL^2-rp.'*rp);
Lz            = diag(Lz).';
Lvec          = [rp;Lz]; % load pos. w.r.t quad pos.
Xp            = Xq + Lvec; % load position over p horizon

Xo            = obstacle.detectedXo; 
[~,nXo]       = size(obstacle.detectedXo); % number of obstacles
[~,p_hor]     = size(Xq);
distQuad2Obs  = zeros(p_hor, nXo);
distLoad2Obs  = zeros(p_hor, nXo);
distCable2Obs = zeros(p_hor, nXo);

for i = 1:nXo
    % Point to ellipsoid distance equation (Uteshev and Goncharova 2018)
    [SoVec,~]          = obstacleGeometry(obstacle,Xo(:,i));
    sigma              = diag([1/SoVec(1)^2,1/SoVec(2)^2,1/SoVec(3)^2]);
    distQuad2Obs(:,i)  = diag(((Xq+eye(3,1)*params.rc)-Xo(:,i)).'*sigma*((Xq+eye(3,1)*params.rc)-Xo(:,i)))-1;    
    distLoad2Obs(:,i)  = diag(((Xp+eye(3,1)*params.rc)-Xo(:,i)).'*sigma*((Xp+eye(3,1)*params.rc)-Xo(:,i)))-1;
    % distCable2Obs(i,:) = 0; 
end

% Put together inequality constraints (<= 0)
distIneqEqns = 3*nXo*p_hor;
cineq_ = zeros(distIneqEqns,1);

idx = 0;
for i = [distQuad2Obs,distLoad2Obs]
    idx = idx + 1;
    start_idx = p_hor*(idx-1)+1;
    end_idx = p_hor*idx;
    cineq_(start_idx:end_idx) = i;
end

cineq = [cineq_;
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
end