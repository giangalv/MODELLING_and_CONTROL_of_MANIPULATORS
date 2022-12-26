function i_T_j = GetFrameWrtFrame(linkNumber_i, linkNumber_j,biTei)
%%% GetFrameWrtFrame function 

% inputs : 
% linkNumber_i : number of ith link 
% linkNumber_j: number of jth link 
% biTei vector of matrices containing the transformation matrices from link i to link i + 1 for the current q.
% The size of biTri is equal to (4,4,numberOfLinks)

% outputs:
% i_T_j : transformation Matrix in between link i and link j for the configuration described in biTei

    i_T_j = eye(4);
    if linkNumber_i <= linkNumber_j
        for i=linkNumber_i:linkNumber_j
            i_T_j= i_T_j * biTei(:, :, i);
        end
    else
        for i=linkNumber_j:linkNumber_i
            i_T_j= biTei(:, :, i) * i_T_j;
        end
    end
end
