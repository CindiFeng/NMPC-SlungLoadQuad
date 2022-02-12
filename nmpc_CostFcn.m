%% stageCostFcn:
%   Functionality    : Define stage and terminal cost functions
%   Inputs           : X                    - states
%                      U                    - input
%                      e                    - slack
%                      data                 - additional signals
%                      obstacle,lim,params  - optional parameters
%   Outputs          : J (minimizing function)

function J = nmpc_CostFcn(X,U,e,data,obstacle,lim,params)
p = data.PredictionHorizon;

Xq = [X(2:p,1).';X(2:p,2).';X(2:p,3).']; % quad position over p horizon
% rp = [X(2:p,13).';X(2:p,14).'];
% Lz = sqrt(params.cableL^2-rp.'*rp);
% Lz = diag(Lz).';
% Lvec = [rp;Lz]; % load pos. w.r.t quad pos.
% Xp = Xq + Lvec; % load position over p horizon

%% Point to point navigation (terminal cost only)
qpos = Xq(:,end); % only position at last time step is used
% cnav_num = ((params.qGoal-qpos).') * eye(3) * (params.qGoal-qpos);
% cnav_den = ((params.qGoal-params.qStart).') * eye(3) * (params.qGoal-params.qStart);
cnav_num = (norm(params.qGoal-qpos))^2;
cnav_den = (norm(params.qGoal-params.qStart))^2;
cnav = cnav_num / cnav_den; % navigation cost term
wnav = 1; % weight

%% Potential field based obstacle separation
% Xo = obstacle.Xo; 
% nXo = obstacle.nXo;
% 
% if nXo == 0
%     cpf = 0;
% else
%     doXq = zeros(nXo,1);
%     doXp = zeros(nXo,1);
%     docp = zeros(nXo,1);
%     cpf = zeros(nXo,1);
%     wpf = zeros(nXo,1);
%     
%     for i = 1:nXo
%         % Point to ellipsoid distance equation (Uteshev and Goncharova 2018
%         [~,sigma] = nmpc_obstacleEllipsoid(obstacle);
%         idx1 = 3*i - 2;
%         idx2 = 3*i;
%         distQ = Xq + eye(3,1)*params.rc - Xo(idx1:idx2,:);
%         distL = Xp + eye(3,1)*params.rc - Xo(idx1:idx2,:);
%         doXq(i) = sum(diag(distQ.' * sigma * distQ)-1);    
%         doXp(i) = sum(diag(distL.' * sigma * distL)-1);
%         docp(i) = 0;
%         
%         if ((doXq < 0) || (doXp < 0) || (docp < 0))
%             cpf(i) = max(abs([doXq,doXp,docp]));
%             wpf(i) = 1.2;
%             disp('Obstacle getting close')
%         end
%     end
% end

%% Input magnitude regulation
inputVec = [U(1:p,1);
            U(1:p,2);
            U(1:p,3);
            U(1:p,4)];
        % U(1:p,data.MVIndex(4))
cin = (norm(inputVec))^2;
win = 0.01;

%% Payload suspension angles regulation
% phi_p = atan(rp(1)./Lz);
% theta_p = atan(rp(2)./Lz);
% cswing = (norm([theta_p;phi_p]))^2;
% wswing = 0.001;

%% Minimization function 
% J = wnav*cnav + win*cin + wswing*cswing + wpf.'*cpf;
J = wnav*cnav + win*cin;