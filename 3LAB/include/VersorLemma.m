function rho = VersorLemma(R1, R2)
%%% VersorLemma function

% Inputs:
% R1: the first rotation matrix;
% R2: the second rotation matrix;

% Output:
% rho: the Vect3 representing the axis around which R1 should rotate to 
%         reach R2, where its modulus is the angle;

    % small threshold to take into account numerical errors in the evaluation of cos_th
    err_threshold = 0.0001;
    
    % inverse-equivalent angle-axis algorithm to compute the misalignment vector
    cos_th = 1/2 * (sum(sum(R1.*R2)) - 1);

    rosinth = zeros(3,1);
    for i = 1:3
        rosinth = rosinth + cross(R1(:,i), (R2(:,i)));
    end

    rosinth = 1/2 * rosinth;
    sin_th = norm(rosinth);

    if (sin_th > (0.1 - err_threshold))
        % Case 0 < th < 180 degrees
        th = atan2(sin_th, cos_th);
        rho = (rosinth * (th / sin_th));

    elseif (cos_th >= err_threshold)
        % Case when cos(th) is almost 1 (no rotation, th = 0)
        rho = [0 ; 0 ; 0];
    else
        % Case when cos(th) is almost -1 (flip around a single axis th = pi)
        R = R1 + R2;
        
        %maxNorm
        [~, idx] = max([norm(R(:,1)), norm(R(:,2)), norm(R(:,3))]);
        maxRho = R(:,idx);
        if (norm(maxRho) ~= 0.0)
            rho = (maxRho * (pi/ norm(maxRho)));
        else
            rho = zeros(3,1);
        end
    end
end

