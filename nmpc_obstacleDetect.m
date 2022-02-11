%% Obstacle Detection function:
%   Functionality    : Detect incoming obstacles and updates obstacle
%                      struct with detected obstacle positions
%   Inputs           : X            - current state
%                      obstacle     - obstacle struct
%                      cableL       - cable length
%   Outputs          : detection    - True/False array to indicate if each 
%                      obstacle is detected

function detection = nmpc_obstacleDetect(X,obstacle,cableL)
% Detect when the vehicle sees an obstacle

Xq      = [X(1);X(2);X(3)]; % current quad position
rp      = [X(13);X(14)];
Lz      = sqrt(cableL^2-rp.'*rp);
Lvec    = [rp;Lz]; % load pos. w.r.t quad pos.
Xp      = Xq + Lvec; % current load position
Xcp     = 0; % cable critical point

Xo      = [obstacle.x;obstacle.y;obstacle.z]; % obstacle positions
[~,nXo] = size(Xo);
detection = zeros(nXo);

for i = 1:nXo
    % Euclidian norm distance to obstacle
    doq = norm(Xq - Xo(:,i));% quadrotor
    dop = norm(Xp - Xo(:,i));% payload
    docp = 100;
    %docp = norm(Xcp - Xo(:,i));% cable
    
    if ((doq <= obstacle.detectionDist)||(dop <= obstacle.detectionDist)||...
        (docp <= obstacle.detectionDist))        
        detection(i) = 1;
    end
end

obstacle.detectedXo = zeros(3,sum(detection));
if sum(detection) > 0
    % Populate detected obstacle positions
    for i = 1:max(size(detection))
        if detection(i)
            obstacle.detectedXo(:,i) = [obstacle.x(i);obstacle.y(i);obstacle.z(i)];                
        end
    end
end