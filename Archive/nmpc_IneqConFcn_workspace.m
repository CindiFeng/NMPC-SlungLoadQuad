%% Workspace inequality constraint function:
%   Functionality    : For confined operation, the quadrotor and load pos.
%                      are limited to workspace limits (assume cuboid).
%   Inputs           : req'd inputs and optional parameters
%   Outputs          : inequality constraint eqns
function cineq = nmpc_IneqConFcn_workspace(X,~,~,data,~,lim,params)

p = data.PredictionHorizon;

Xq            = [X(2:p+1,1).';X(2:p+1,2).';X(2:p+1,3).']; % quad position over p horizon

rp            = [X(2:p+1,13).';X(2:p+1,14).'];
Lz            = sqrt(params.cableL^2-rp.'*rp);
Lz            = diag(Lz).';
Lvec          = [rp;Lz]; % load pos. w.r.t quad pos.
Xp            = Xq + Lvec; % load position over p horizon

cineq = [Xp-lim.Wmax;
         -Xp+lim.Wmin;
         Xq-lim.Wmax;
         -Xq+lim.Wmin];
end