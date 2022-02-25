%% Obstacle Detection function:
%   Functionality    : Detect incoming obstacles and updates obstacle
%                      struct with detected obstacle positions
%   Inputs           : X            - current state
%                      obstacle     - obstacle struct
%                      cableL       - cable length
%   Outputs          : detection    - True/False array to indicate if each 
%                      obstacle is detected

function detection = ms_obstacleDetect(Xq,Xp,Xcp,obstacle)
% Detect when the vehicle sees an obstacle
Xo      = [obstacle.x; obstacle.y; obstacle.z]; % obstacle positions [x y z]'
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