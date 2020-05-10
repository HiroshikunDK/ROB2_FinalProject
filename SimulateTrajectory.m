%% Get route
OrgWP = [1.0 1.0; 2.0 4.0;5.0 4.0;6.0 3.0;8.0 8.0];
WayPoints = OrgWP;
currentPos = [0.5 0.5];
Orientation = 0;

%% Angle to watch 
%21.83 degres from center 
%0.9283 radinas from center
% if these are under 50 cm, then turtlebot wont fit between them.
%y=asind(2.5);


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

for n = 1 : length(WayPoints)
    [currentPos, Orientation,isAtEnd]=MoveP2PSim(robot, currentPos, Orientation, controller, WayPoints(n),OrgWP,0.1);
    if(isAtEnd==0)
        deltapos=bugaroundRight();
        disp(deltapos)
        currentPos(1)=currentPos(1)+deltapos(1);
        currentPos(2)=currentPos(2)+deltapos(2);
        currentPos(3)=deltapos(3);
        %reset the path during execution
        %WayPoints2 = [1.0 1.0; 2.0 4.0; 3.0 3.0; 4.0 3.0; 4.0 4.0;5.0 4.0;6.0 3.0;8.0 8.0];
        %WayPoints = WayPoints2;
        %controller = createController(WayPoints,0.1,0.5,0.5);
        %disp("Controller waypoints set")
        %disp(lcisstoo)
    end

end     
%%


function controller = createController(WayPoints,DLV,MAV, LAD) 
controller = controllerPurePursuit;
controller.Waypoints = WayPoints;
controller.DesiredLinearVelocity = DLV;
controller.MaxAngularVelocity = MAV;
controller.LookaheadDistance = LAD;

end 

%% Model Trajectory simulation
function bool = resetODOM()


    velpub = rospublisher("/mobile_base/commands/reset_odometry");
    msg =rosmessage('std_msgs/Empty');
    send(velpub,msg);
    pause(3);
    disp("reseODOM called")
    
    bool = 1;

end


function scandata = get2DScan()

    scanlive = rossubscriber("/scan");
    scandata2 = receive(scanlive,10);
    disp("Scan OK")

end 

