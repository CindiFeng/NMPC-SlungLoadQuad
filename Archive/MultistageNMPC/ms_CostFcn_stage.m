%% stageCostFcn:
%   Functionality    : Define stage and terminal cost functions
%   Inputs           : X                    - states
%                      U                    - input
%                      e                    - slack
%                      data                 - additional signals
%                      obstacle,lim,params  - optional parameters
%   Outputs          : J (minimizing function)

function cost = ms_CostFcn_stage(stage,x,u,p)
obstacle = ms_obstacleTraj(stage);
rc = p(1);
cableL = p(2);

Xq = [x(1);x(2);x(3)]; % quad position over p horizon
rp = [x(13);x(14)];
Lz = sqrt(cableL^2-rp.'*rp);
Lz = diag(Lz).';
Lvec = [rp;Lz]; % load pos. w.r.t quad pos.
Xp = Xq + Lvec; % load position over p horizon

%% Potential field based obstacle separation
Xo = obstacle.Xo; 
nXo = obstacle.nXo;

if nXo == 0
    cpf = 0;
else
    doXq = zeros(nXo,1);
    doXp = zeros(nXo,1);
    docp = zeros(nXo,1);
    cpf = zeros(nXo,1);
    wpf = zeros(nXo,1);
    
    for i = 1:nXo
        % Point to ellipsoid distance equation (Uteshev and Goncharova 2018
        [~,sigma] = nmpc_obstacleEllipsoid(obstacle);
        idx1 = 3*i - 2;
        idx2 = 3*i;
        distQ = Xq + eye(3,1)*rc - Xo(idx1:idx2,stage);
        distL = Xp + eye(3,1)*rc - Xo(idx1:idx2,stage);
        doXq(i) = sum(diag(distQ.' * sigma * distQ)-1);    
        doXp(i) = sum(diag(distL.' * sigma * distL)-1);
        docp(i) = 0;
        
        if ((doXq < 0) || (doXp < 0) || (docp < 0))
            cpf(i) = max(abs([doXq,doXp,docp]));
            wpf(i) = 1.2;
            disp('obstacle getting close')
        end
    end
end

%% Input magnitude regulation
inputVec = [u(1);
            u(2);
            u(3);
            u(4)];
cin = norm(inputVec)^2;
win = 0.01;

%% Payload suspension angles regulation
phi_p = atan(rp(1)./Lz);
theta_p = atan(rp(2)./Lz);
cswing = norm([theta_p;phi_p])^2;
wswing = 0.001;

%% Minimization function 
cost = win*cin + wswing*cswing + wpf.'*cpf;
