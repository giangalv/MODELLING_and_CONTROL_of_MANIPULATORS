function [skew_matrix] = Skew_change(v)
% v is a vector 1x3 
%INPUT: v -> vector 1x3
%OUTPUT: skew_matrix -> matrix like v_^
skew_matrix = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end
