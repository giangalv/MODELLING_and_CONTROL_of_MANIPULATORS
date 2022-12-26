%% Modelling and Control of Manipulator assignment 2: Manipulator Geometry and Direct kinematic
clc;
clear;
close all;
addpath('include');

%% 1.
% You will need to define all the model matrices, and fill the so called iTj matrices inside BuildTree() function 
% Be careful to define the z-axis coinciding with the joint rotation axis,
% and such that the positive rotation is the same as showed in the CAD model you received.
%% Q1.1
% Useful initizializations
linkType = [0, 0, 0, 0, 0, 0, 0];                % boolean that specifies two possible link types: Rotational, Prismatic.                                                
numberOfLinks = length(linkType);                % number of manipulator's links.
bRi= zeros(3,numberOfLinks);                     % Basic vector of i-th link w.r.t. base
bTi = zeros(4,4,numberOfLinks);                  % Trasformation matrix i-th link w.r.t. base

geom_model = BuildTree();

% Initial joint configuration  
q = [2, 2, 2, 2, 2, 2, 2];

%% Q1.2
% Calling GetDirectGeometry(), which returns all the model matrices given
% a certain q configuration.
biTei = GetDirectGeometry(q, geom_model, linkType);

%% Q1.3
% compute the transformation matrix of the joints w.r.t. the base
for i = 1:numberOfLinks
    bTi(:,:,i)= GetTransformationWrtBase(biTei, i);
end

% compute the transformation matrix of the a joints w.r.t. the joint b
% you can choose the two links with a and b.
a = 1; % i link
b = 2; % j link
iTj = GetFrameWrtFrame(a, b, biTei);


% get the vectors that show the position of the joints <i> and the last
% joint w.r.t. the base in order 
for i = 1:numberOfLinks
    bRi(:,i) = GetBasicVectorWrtBase(biTei, i);
end

%% Q1.4 DIRECT KINEMATIC SIMULATION
% initial condition: 
qi1 = q;
qi2 = [1.3, 0, 1.3, 1.7, 1.3, 0.8, 1.3];
qi3 = [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3];
qi = [qi1', qi2', qi3'];

% final condition:
qf1 = [2, 2, 2, 2, 2, 2, 2];
qf2 = [2, 0, 1.3, 1.7, 1.3, 0.8, 1.3];
qf3 = [2, 2, 2, 2, 2, 2, 2];
qf = [qf1', qf2', qf3'];

numberOfSteps = 100; 

for n = 1:3
    PlotConfigurationDirectKinematic(numberOfSteps, qi(:,n), qf(:,n), geom_model, linkType, n)
end

%% Q1.5 MOVEMENT OF ONE JOINT POSITION
qi5 = qi(:, 1);
qf5 = qi(:, 1);
end_program = "N";
while end_program == "N" || end_program == "n"
    confirm = "n";
    while confirm ~= "Y" && confirm ~= "y"
        fprintf("\n\n What number of joint do you want to change the configuration? ");
        n_joint = input(' ');
        fprintf(" Do you confirm that the choice number %d? ", n_joint);
        confirm = input(' [Y/n] ', 's');
    end
    confirm = "n";
    while confirm ~= "Y" && confirm ~= "y"
        fprintf("\n\n What q value do you want for the %d joint? ", n_joint);
        m = input(' ');
        fprintf(" Do you confirm that the choice number %d? ", m);
        confirm = input(' [Y/n] ', 's');
    end
    qf5(n_joint) = m;
    PlotConfigurationDirectKinematic(numberOfSteps, qi5, qf5, geom_model, linkType, 1);
    qi5 = qf5;
    confirm = "n";
    fprintf("\n\n Do you want to exit? ");
    end_program = input(' [Y/n] ', 's');
end