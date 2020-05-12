function [endPos, Orientation, isAtEnd] = MoveP2P(RobObj, initialPos, initOrientation, ControllerObj, EndPoint, WayPoints, SampleTime)

%robot intialisation.
%robotInitialLocation = initialPos; %get from robot
robotGoal = EndPoint;
robotCurrentPose = [initialPos initOrientation];


distanceToGoal = norm(initialPos - robotGoal);

goalRadius = 0.2;

% Initialize the simulation loop
vizRate = rateControl(1/SampleTime);

% Initialize the figure
%figure

 
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [i, omega] = ControllerObj(robotCurrentPose(1,:));
    release(ControllerObj)
    
    % Get the robot's velocity using controller inputs
    vel = derivative(RobObj, robotCurrentPose(1,:), [i, omega]);
    
    bool=Drive(ControllerObj.DesiredLinearVelocity, vel(3,1));
    
    %calculate the new robot position.
    robotCurrentPose = robotCurrentPose + (vel*SampleTime)'; 
    
    if(scanForObstacles()==1)
        disp("Interrupt Statement triggered") 
        
        endPos=robotCurrentPose(1,[1 2]);
        Orientation=robotCurrentPose(1,3);
        isAtEnd = 0;
        return;
    end 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1,1:2) - robotGoal(:));
      
    % Plot the path of the robot as a set of transforms
    %hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    %plot(WayPoints(:,1), WayPoints(:,2),"k--d")
    %xlim(PlotLimit2D)
    %ylim(PlotLimit2D)
    %hold all
    
    %draw the new robot postion and orientation.
    %plotTrVec = [robotCurrentPose(1:2) 0];
    %plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    %plotTransforms(plotTrVec, plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    %light;
    
    waitfor(vizRate);
end
    bool=0;
    endPos = robotCurrentPose(1:2);
    Orientation = robotCurrentPose(3);
    isAtEnd =1;    
end
