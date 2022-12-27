function [] = PlotConfigurationDirectKinematic(numberOfSteps, qi, qf, geom_model, linkType, n)
%%% PlotConfigurationDirectKinematic function

% Inputs:
% numberOfSteps : steps'number to pass from qi to qf
% qi : current robot configuration
% qf : future robot configuration
% geom_model : the tree of frames
% linkType : specifies two possible link types: Rotational, Prismatic
% n : plot number (it is usefull to print in a for loop)

% Output:
% the figure with the robot and its movements from qi to qf
    
    numberOfLinks = length(linkType);

    % For each single link we want to separate the path in n steps.
    qSteps = [linspace(qi(1),qf(1),numberOfSteps)',...
              linspace(qi(2),qf(2),numberOfSteps)',...
              linspace(qi(3),qf(3),numberOfSteps)',...
              linspace(qi(4),qf(4),numberOfSteps)',...
              linspace(qi(5),qf(5),numberOfSteps)',...
              linspace(qi(6),qf(6),numberOfSteps)',...
              linspace(qi(7),qf(7),numberOfSteps)'];
    
    % Plotting the movement of the manipulator.
    figure
    title("Manipulator's motion n°: ", n);
    xlabel('x')
    ylabel('y')
    zlabel('z')
    grid on
    axis equal
    hold on

    % Defining the point of view of the view
    azimuth = 50;
    elevation = 25;
    view(azimuth,elevation)
    
    % Plotting all the frames through a for loop.
    for i = 1:numberOfSteps
        
        % Initialazing the vector matrix including the base (0; 0; 0).
        bRij = zeros(3,numberOfLinks+1); 
    
        q = qSteps(i,1:numberOfLinks)';
    
        % Filling the bitei matrix using the function GetDirectGeometry().
        biTei = GetDirectGeometry(q, geom_model, linkType);
        
        % Extrapolating basic vector with respect to the base from biTei.
        % taking into account the presence of the base (0,0,0).
        for j = 1:numberOfLinks
            
            bRij(:,j+1) = GetBasicVectorWrtBase(biTei,j);
    
        end
        
        % plotting the joints
        for j = 2:numberOfLinks+1
            
           plot3(bRij(1,j),bRij(2,j),bRij(3,j),'k.', 'MarkerSize', 15)
           
        end
    
        % plotting the lines connecting all the joints. 
        line(bRij(1,:),bRij(2,:),bRij(3,:),'LineWidth',3,'Color', '#4DBEEE')
    
        % plotting the base in (0,0,0).
        plot3(0,0,0,'red.','MarkerSize', 30)
    
        hold on
        getframe;
        
        % refreshing the window every loop-1.
        if i < numberOfSteps 
            % function that clean the window.
            cla();
        end
    end
    % plot now
    drawnow
end

