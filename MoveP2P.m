function [newPose, isAtEnd] = MoveP2P(RobObj, initialPos, ControllerObj, EndPoint, SampleTime)

%robot intialisation.
%robotInitialLocation = initialPos; %get from robot
robotGoal = EndPoint;
robotCurrentPose = initialPos;
startOdomPose = getCurrentPos();

distanceToGoal = norm(initialPos(1:2) - robotGoal);

goalRadius = 0.2;

% Initialize the simulation loop
vizRate = rateControl(1/SampleTime);

% Initialize the figure
%figure
startTime=vizRate.TotalElapsedTime;
 
while( distanceToGoal > goalRadius )
    
    %robotCurrentPose = robotCurrentPose - startOdomPose;
    % Compute the controller outputs, i.e., the inputs to the robot
    [i, omega] = ControllerObj(robotCurrentPose(1,:));
    release(ControllerObj)
    
    % Get the robot's velocity using controller inputs
    vel = derivative(RobObj, robotCurrentPose(1,:), [i, omega]);
    move(ControllerObj.DesiredLinearVelocity, vel(3,1));
    
    %calculate the new robot position.
    tempPose = getCurrentPos()-startOdomPose;
    robotCurrentPose = tempPose + initialPos;    
    currentTime = vizRate.TotalElapsedTime;

    %interrupt settings
    if((currentTime - startTime)>4)
        startTime=startTime + 4;
        scandata2 = get2DScan();
        [Right_angle,Right_range,Left_angle,Left_range,isItBlocked] = scanForObstacles(scandata2);
    
        if(isItBlocked==1)
            disp("Interrupt Statement triggered")
            newPose = robotCurrentPose;
            isAtEnd=0;
            
            return;
        end 
    end
    
    waitfor(vizRate);
end    
    newPose = robotCurrentPose;
    isAtEnd =1;    
end
