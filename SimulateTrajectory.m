%% Get route
OrgWP = [1.0 1.0; 2.0 4.0;5.0 4.0;6.0 3.0;8.0 8.0];
WayPoints = OrgWP;
currentPos = [0.5 0.5];
Orientation = 0;

%% Angle to watch 
%21.83 degres from center 
%0.9283 radinas from center
% if these are under 50 cm, then turtlebot wont fit between them.
y=acosd(0.9283);


%% Setup

%controller = controllerPurePursuit
%controller.Waypoints = WayPoints
%controller.DesiredLinearVelocity = 0.1;
%controller.MaxAngularVelocity = 0.5;
%controller.LookaheadDistance = 0.5;

controller = createController(WayPoints,0.3,0.5,0.5);

robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

%% run function 

%resetODOM();

%frameSize =MakePlot(WayPoints);

%distanceToGoal = norm(currentPos(1,1:2) - robotGoal(:));
goalRadius = 0.2;

lcisstoo = 0;

for n = 1 : length(WayPoints)
    [currentPos, Orientation, lcisstoo,isAtEnd]=MoveP2PSim(robot, currentPos, Orientation, controller, WayPoints(n), lcisstoo,OrgWP,0.1);
    if(isAtEnd==0)
        %bug around
        WayPoints2 = [1.0 1.0; 2.0 4.0; 3.0 3.0; 4.0 3.0; 4.0 4.0;5.0 4.0;6.0 3.0;8.0 8.0];
        WayPoints = WayPoints2;
        controller = createController(WayPoints,0.1,0.5,0.5);
        disp("Controller waypoints set")
        disp(lcisstoo)
    end

end     
%%
function overshotEndPoint = bugaround(currentPos)
   

end

function controller = createController(WayPoints,DLV,MAV, LAD) 
controller = controllerPurePursuit;
controller.Waypoints = WayPoints;
controller.DesiredLinearVelocity = DLV;
controller.MaxAngularVelocity = MAV;
controller.LookaheadDistance = LAD;

end 

%% Is there an obstacle?
function [width, isItBlocked] = scanForObstacles(RobObj)
%Take and analyse the scaline here.

%should return the approx length of the object, so we know where to go past
%it 
width = 0;
% should be 1 if obstacle is detected, within the threshold.
isblocked = 0; 
end 
%% Model Trajectory simulation
function [endPos, Orientation,bool, isAtEnd] = MoveP2PSim(RobObj, initialPos, initOrientation, ControllerObj, EndPoint,lcisstoo, WayPoints, SampleTime)

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


%robot intialisation.
%robotInitialLocation = initialPos; %get from robot
robotGoal = EndPoint;
robotCurrentPose = [initialPos initOrientation];


distanceToGoal = norm(initialPos - robotGoal);

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
    
    if((robotCurrentPose(1,1)>3)&&(lcisstoo==0))
        disp("Interrupt Statement triggered") 
        bool=1;
        endPos=robotCurrentPose(1,[1 2]);
        Orientation=robotCurrentPose(1,3);
        isAtEnd = 0;
        return;
    end 
    
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
    bool=0;
    endPos = robotCurrentPose(1:2);
    Orientation = robotCurrentPose(3);
    isAtEnd =1;    
end

function bool = Drive(X,Omega)

    vel = X; %meters per second
    velpub = rospublisher("/mobile_base/commands/velocity");
    velmsg = rosmessage(velpub);

    velmsg.Linear.X = vel;
    velmsg.Angular.Z = Omega/2;
    send(velpub,velmsg);
    bool = 1;

end

function bool = resetODOM()


    velpub = rospublisher("/mobile_base/commands/reset_odometry");
    msg =rosmessage('std_msgs/Empty');
    send(velpub,msg);
    pause(3);
    disp("reseODOM called")
    
    bool = 1;

end

function pos= getCurrentPos()
odom = rossubscriber("/odom");

odomdata = receive(odom,3);

pose = odomdata.Pose.Pose;

x = pose.Position.X;
y = pose.Position.Y;
quat = pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%theta = rad2deg(angles(1))
Omega = angles(1);
pos= [x y Omega];
end 

function [range, angle] = getAngleRange(scan,rads)

    
    %Use this section for demo data
    cart1 = readCartesian(scan);
    
    x = cart1(:,2); % x-pos
    d = cart1(:,1); % depth
    
    xslice = [];
    dslice = [];
    
    plot(x,d, '.'), hold on
    ylim([0 3])
    xlim([-0.7 0.7])
    
    lastVisit = -1;
    cnt=0;
    
    for j = 1:1:length(x)

        if (x(j)>(rads-0.005))&&(x(j)<(rads+0.005))
            %fjern comment for at se hvilke xværdier er brugt. 
            %disp(x(j))
            lastVisit = j;
            cnt = cnt + 1;
        end
    end
    
    %disp("lastVisit: ")
    %disp(lastVisit)
    %disp("count: ")
    %disp(cnt)    
    
    assert(lastVisit ~= -1)
    
    xslice = x((lastVisit-cnt):lastVisit)
    dslice = d((lastVisit-cnt):lastVisit)
    
    %disp(lastVisit-cnt-10)
    
    mdl = fitlm(xslice,dslice);
    coef=mdl.Coefficients.Estimate;
    
    xP = [(rads-0.2),(rads+0.2)]
    
    plot(xP, coef(1) + coef(2)*xP, 'r')
        
    range = coef(2)*rads + coef(1)
    angle = rad2deg(atan(coef(2)))
    
end
