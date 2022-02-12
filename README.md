# NMPC-SlungLoadQuad
>Online trajectory planning and control using nonlinear MPC model in MATLAB to avoid dynamic/static obstacles

## Function
- main.m - run file containing NMPC parameters and specifications
- CostFcn.m - cost equations following Potdar et al paper; terminal cost term included using only the last predicted state 
- IneqConFcn_collisionFree.m - constraints equations followingPotdar et al paper; output is a huge column vector
- StateFcn.m - contains EOM and outputs dx/dt
- obstacleTraj.m - generates obstacle trajectory based on time; currently set as static obstacle (no velocity/acceleration terms)
- obstacleEllipsoid.m - turns user-specified cuboid obstacle dimensions to bounding ellipsoid dimensions to be used in obstacle avoidance equations 
- referenceTrajectory.m - writes custom reference trajectory to be tracked for ALL states across the simulation time
- CostFcn_terminal.m - used for MSNMPC as terminal cost equation
- CostFcn_stage.m - used for MSNMPC as stage cost equations
- (archived) obstacleDetect.m - tracks # of detected obstacles and their locations
- (archived) obstacleGeometry.m - same as obstacleEllipsoid.m without bug fixes
