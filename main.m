%% Setup Kobuki Enviorment - Laptop

setenv('ROS_MASTER_URI','http://192.168.128.129:11311')

setenv('ROS_IP','192.168.100.105')

rosinit('http://192.168.128.129:11311','NodeHost','192.168.100.105');

%% Setup Kobuki Enviorment - HomeStation: 
setenv('ROS_MASTER_URI','http://192.168.1.164:11311')

setenv('ROS_IP','192.168.1.92')

rosinit('http://192.168.1.164:11311','NodeHost','192.168.1.92');

%% Set Up for Main Section

% Original waypoints saved to plot original route 
OrgWP = [1.0 1.0; 2.0 4.0;8.0 4.0;8.0 2.0;4.0 4.0];
WayPoints = OrgWP;
currentPos = [0.0 0.0];

%Controllers used to calulate angular velocity based on position on
%trajectory
%createController(WayPoints,DesiredLinearVelocity,MaximumAngularVelocity, LookAheadDistance)
controller = createController(WayPoints,0.25,0.4,0.35);
controllerCrawl = createController(WayPoints,0.15,0.5,0.25);

%dunno but needed
robot1 = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

%% this section is used if the robot doesn't move   
   
    velpub = rospublisher("/mobile_base/commands/velocity");
    velmsg = rosmessage(velpub)

    velmsg.Linear.X = 1;
    velmsg.Linear.Y = 0;
    velmsg.Linear.Z = 0;
    velmsg.Angular.X = 0;
    velmsg.Angular.Y = 0;
    velmsg.Angular.Z = 0;
        
    send(velpub,velmsg);
    
%% Test bugaround 

globalpos=[0,0,0];

sdtest=get2DScan();
deltaPos1 = move2obj(globalpos,0.7,sdtest)
newcontroller = createController([deltaPos1(1:2)],0.15,0.5,0.25);
[globalpos,isAtEnd]=MoveP2P(robot1, globalpos, newcontroller, deltaPos1(1:2),0.1);
disp("Done")

%% test image analysis

%reset all robot info
globalPos = [0, 0, -1];

deltaPos = calcBugAroundPos(globalPos,-90, 0.7);

%globalPos = bugaround(robot1, 0.1, globalPos)
%disp("Bugaround ended!!!!")
%Twist(3.141/2);


%% Main
disp("Test function")

%Global position can be set here.
currentPos = [0.0 0.0];

%should help isolate problems about position but should not be needed
resetODOM();

PlotWayPoints(OrgWP)

%start following the route defined in the controller. 
for n = 1 : length(WayPoints)
    
    %will follow to the defined endpoint of not obstructed WayPoints(n)
    [currentPos,isAtEnd]=MoveP2P(robot1, currentPos, controller, WayPoints(n),0.1);
    
    %obstruction is detected for every 4th second 
    if(isAtEnd==0)
        %enter different mode to evade obstructions
        currentPos = bugaround(robot1, 0.1, currentPos)
        disp("Bugaround ended!!!!")  
    end
    disp("Waypoint reached: ")
    disp(n)
end
