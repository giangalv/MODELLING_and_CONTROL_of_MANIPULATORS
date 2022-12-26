function R = ComputeElementaryOrientationMatrix(v,theta)
% implement the basic rotation, in three dimensions, about one of the axes
% of a coordinate system.
if v == [1 0 0]
    R = [1, 0, 0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];
elseif v == [0 1 0]
    R = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0 cos(theta)];
elseif v == [0 0 1]
    R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
else
    R = [0,0,0;0,0,0;0,0,0];
end
end