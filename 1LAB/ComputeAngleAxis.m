function R = ComputeAngleAxis(theta,v)
%Implement here the Rodrigues formula
R = eye(3) + (Skew_change(v)*sin(theta)) + ((Skew_change(v))^2 *(1 - cos(theta)));
end