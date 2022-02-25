%% Euler Angle Rotation Matrix:
%   Functionality    : Called to convert body rotating axes to 
%                      Euler angles
%   Inputs           : Euler and body angular rates
%   Outputs          : Rotation matrix
function euler_rates = util_eulerRotMat(phi,theta,p,q,r)
    Sb = [1 0 -sin(theta)
          0 cos(phi) sin(phi)*cos(theta)
          0 -sin(phi) cos(phi)*cos(theta)]; 
    wb = [p;q;r];
    euler_rates = Sb\wb;
end