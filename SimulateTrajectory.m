%% new stuff

    
    
%% make plot
function frameSize = MakePlot(WayPoints)

%plot borders and points
%if both axis' lowest value is over 0
if (min(WayPoints(:,1)) > 0) && (min(WayPoints(:,2)) > 0)
    if max(WayPoints(:,1)) > max(WayPoints(:,2))
    
        disp("Version 1: plot size based on X axis")    
  
        PlotLimit2D = [0 (max(WayPoints(:,1))*1.5)]
    
        % TrackWidth per default 1, so become 5% of the size of the x border.
        frameSize = PlotLimit2D(2)*0.05;
    else 
        disp("Version 2: plot size based on Y axis")    
  
        PlotLimit2D = [0 (max(WayPoints(:,2))*1.5)]
    
        % TrackWidth per default 1, so become 5% of the size of the x border.
        frameSize = PlotLimit2D(2)*0.05;
    end 
else
    if max(abs(diff(WayPoints(:,1)))) > max(abs(diff(WayPoints(:,2))))
    
        disp("Version 3: plot axis in negative quadrant")    
  
        PlotLimit2D = [(min(WayPoints(:,1))*1.5) min(WayPoints(:,1))+(max(abs(diff(WayPoints(:,1))))*1.5)]
    
        % TrackWidth per default 1, so become 5% of the size of the x border.
        frameSize = max(abs(diff(WayPoints(:,1))))*0.05;
    else 
        disp("Version 4: plot axis in negative quadrant")    
  
        PlotLimit2D = [(min(WayPoints(:,2))*1.5) min(WayPoints(:,2))+(max(abs(diff(WayPoints(:,2))))*1.5)]
    
        % TrackWidth per default 1, so become 5% of the size of the x border.
        frameSize = max(abs(diff(WayPoints(:,2))))*0.05;
    end
end
end 
    
%% Model Trajectory simulation
function [endPos, Orientation, isAtEnd] = MoveP2PSim(RobObj, initialPos, ControllerObj, EndPoint, SampleTime, frameSize)


%robot intialisation.
%robotInitialLocation = initialPos; %get from robot
robotGoal = EndPoint;
robotCurrentPose = [initialPos initialOrientation];


distanceToGoal = norm(robotInitialLocation - robotGoal);

goalRadius = 0.2;

% Initialize the simulation loop
vizRate = rateControl(1/SampleTime);

% Initialize the figure
figure

 
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [i, omega] = ControllerObj(robotCurrentPose(1,:));
    release(ControllerObj)
    
    % Get the robot's velocity using controller inputs
    vel = derivative(RobObj, robotCurrentPose(1,:), [i, omega]);
    
    %calculate the new robot position.
    robotCurrentPose = robotCurrentPose + (vel*SampleTime)'; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1,1:2) - robotGoal(:));
      
    % Plot the path of the robot as a set of transforms
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(WayPoints(:,1), WayPoints(:,2),"k--d")
    xlim(PlotLimit2D)
    ylim(PlotLimit2D)
    hold all
    
    %draw the new robot postion and orientation.
    plotTrVec = [robotCurrentPose(1:2) 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec, plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    
    waitfor(vizRate);
end
    bool = 1;
end
