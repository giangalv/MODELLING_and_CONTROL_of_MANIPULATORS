function r = GetBasicVectorWrtBase(biTei, linkNumber)
%%% GetBasicVectorWrtBase function 
% Input :
% iTj trasnformation matrix in between i and j 

% Output
% r : basic vector from i to j

    bTi = GetFrameWrtFrame(1, linkNumber, biTei);
    r = bTi(1:3,4)';
end
