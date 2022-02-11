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

% Obstacle model 
obstacle = struct;
obstacle.detectionDist = 3; 

% Obstacle cuboid dimensions 
obstacle.L = 0.5; 
obstacle.W = 0.5;
obstacle.H = 0.5;

obstacle.xVel = 0.5; % obstacle velocity [m/s]

% Initial obstacle position(s)
obstacle.x = 2; 
obstacle.y = 0; 
obstacle.z = 2;
obstacle.detectedXo = zeros(3,1);

% Buffer distances
obstacle.Bo = 0.05; % bounding ellipsoid for which collisionss are checked
obstacle.Be = 0.2; % expanded ellipsoid identified as high risk zone in planning

%% Determine obstacle distances
rc = p(1);
cableL = p(2);

Xq = [x(1);x(2);x(3)]; % quad position over p horizon
rp = [x(13);x(14)];
Lz = sqrt(cableL^2-rp.'*rp);
Lz = diag(Lz).';
Lvec = [rp;Lz]; % load pos. w.r.t quad pos.
Xp = Xq + Lvec; % load position over p horizon
Xcp = 0;

detection = ms_obstacleDetect(Xq,Xp,Xcp,obstacle);

if sum(detection) > 0
    obstacle.detectedXo = zeros(3,sum(detection));

    % Populate detected obstacle positions
    for i = 1:max(size(detection))
        if detection(i)
            obstacle.detectedXo(:,i) = [obstacle.x(i);obstacle.y(i);obstacle.z(i)];                
        end
    end        
end

Xo            = obstacle.detectedXo;
[~,nXo]       = size(obstacle.detectedXo); % number of deteced obstacle positions
[~,p_hor]     = size(Xq);
distQuad2Obs  = zeros(p_hor, nXo);
distLoad2Obs  = zeros(p_hor, nXo);
distCable2Obs = zeros(p_hor, nXo);

for i = 1:nXo
    % Point to ellipsoid distance equation (Uteshev and Goncharova 2018)
%     [SoVec,~]          = obstacleGeometry(obstacle,Xo(:,i));
    [SoVec,~]          = obstacleGeometry(obstacle);
    sigma              = diag([1/SoVec(1)^2,1/SoVec(2)^2,1/SoVec(3)^2]);
    distQuad2Obs(:,i)  = diag(((Xq+eye(3,1)*rc)-Xo(:,i)).'*sigma*((Xq+eye(3,1)*rc)-Xo(:,i)))-1;    
    distLoad2Obs(:,i)  = diag(((Xp+eye(3,1)*rc)-Xo(:,i)).'*sigma*((Xp+eye(3,1)*rc)-Xo(:,i)))-1;
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
         (Xp(1,:)-Wmax(1)).';
         (Xp(2,:)-Wmax(2)).';
         (Xp(3,:)-Wmax(3)).';
         (-Xp(1,:)+Wmin(1)).';
         (-Xp(2,:)+Wmin(2)).';
         (-Xp(3,:)+Wmin(3)).';
         (Xq(1,:)-Wmax(1)).';
         (Xq(2,:)-Wmax(2)).';
         (Xq(3,:)-Wmax(3)).';
         (-Xq(1,:)+Wmin(1)).';
         (-Xq(2,:)+Wmin(2)).';
         (-Xq(3,:)+Wmin(3)).'];
end