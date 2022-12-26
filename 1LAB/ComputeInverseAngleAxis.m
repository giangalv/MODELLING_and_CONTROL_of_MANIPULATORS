function [theta,v] = ComputeInverseAngleAxis(R)
%EULER REPRESENTATION: Given a tensor rotation matrices this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'v' (axis) 
% SUGGESTED FUNCTIONS
    % size()
    % eye()
    % eig()
    % find()
    % abs()
    % det()
    % NB: Enter a square, 3x3 proper-orthogonal matrix to calculate its angle
    % and axis of rotation. Error messages must be displayed if the matrix
    % does not satisfy the rotation matrix criteria.
    det_R = int8(det(R));
    otogonal_matrix = abs(int8(R * R'));
    % Check matrix R to see if its size is 3x3
    if(size(R) == size(eye(3)))
        % Check matrix R to see if it is orthogonal
         if(otogonal_matrix == int8(eye(3))) 
            % Check matrix R to see if it is proper: det(R) = 1
             if(det_R == 1)
                theta = acos((trace(R)-1) / 2);
                [eigenvectors, eigenvalues] = eig(R);
                for r=1:3
                    for c=1:3
                        if(int8((eigenvalues(r,c))) == 1)
                            v = eigenvectors(:,c);
                        end
                    end
                end
            else
                error('DETERMINANT OF THE INPUT MATRIX IS 0');
            end
        else
            error('NOT ORTHOGONAL INPUT MATRIX');
        end
    else
        error('WRONG SIZE OF THE INPUT MATRIX');
    end
end

