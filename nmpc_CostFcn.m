%% stageCostFcn:
%   Functionality    : Define stage and terminal cost functions
%   Inputs           : X                    - states
%                      U                    - input
%                      e                    - slack
%                      data                 - additional signals
%                      obstacle,lim,params  - optional parameters
%   Outputs          : J (minimizing function)

function J = CostFcn(X,U,e,data,obstacle,lim,params)
p = data.PredictionHorizon;

Xq = [X(2:p,1).';X(2:p,2).';X(2:p,3).']; % quad position over p horizon
rp = [X(2:p,13).';X(2:p,14).'];
Lz = sqrt(params.cableL^2-rp.'*rp);
Lz = diag(Lz).';
Lvec = [rp;Lz]; % load pos. w.r.t quad pos.
Xp = Xq + Lvec; % load position over p horizon

%% Point to point navigation (terminal cost only)
qpos = Xq(:,end); % only position at last time step is used
cnav_num = ((params.qGoal-qpos).') * eye(3) * (params.qGoal-qpos);
cnav_den = ((params.qGoal-params.qStart).') * eye(3) * (params.qGoal-params.qStart);
cnav = cnav_num / cnav_den; % navigation cost term
wnav = 1; % weight

%% Potential field based obstacle separation
% Xo = [obstacle.x; obstacle.y; obstacle.z]; 
% [~,nXo] = size(Xo); % total number of obstacles
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
%         [~,SeVec] = obstacleGeometry(obstacle,obstacle.detectedXo(:,i));
%         sigma = diag([1/SeVec(1)^2,1/SeVec(2)^2,1/SeVec(3)^2]);
%         doXq(i) = sum(diag(((Xq+eye(3,1)*params.rc)-Xo(:,i)).'*sigma*((Xq+eye(3,1)*params.rc)-Xo(:,i))-1));    
%         doXp(i) = sum(diag(((Xp+eye(3,1)*params.rc)-Xo(:,i)).'*sigma*((Xp+eye(3,1)*params.rc)-Xo(:,i))-1));
%         docp(i) = 0;
%         
%         if ((doXq < 0) || (doXp < 0) || (docp < 0))
%             cpf(i) = max(abs([doXq,doXp,docp]));
%             wpf(i) = 1.2;
%             print('Obstacle getting close')
%         end
%     end
% end

%% Input magnitude regulation
inputVec = [U(1:p,1);
            U(1:p,2);
            U(1:p,3);
            U(1:p,4)];
        % U(1:p,data.MVIndex(4))
cin = norm(inputVec)^2;
win = 0.01;

%% Payload suspension angles regulation
% phi_p = atan(rp(1)./Lz);
% theta_p = atan(rp(2)./Lz);
% cswing = norm([theta_p;phi_p])^2;
% wswing = 0.001;

%% Minimization function 
% J = wnav*cnav + win*cin + wswing*cswing + wpf.'*cpf;
J = wnav*cnav + win*cin;