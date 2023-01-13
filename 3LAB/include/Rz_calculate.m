function Rz = Rz_calculate(theta)
%%% Rz_calculate function

% Inputs:
% theta:  angle of rotation around the z-axis;

% Output:
% Rz: rotation matrix defined by the rotation around the z-axis;

    Rz = [ cos(theta) -sin(theta)  0;
           sin(theta)  cos(theta)  0;
              0        0       1; ];
end
