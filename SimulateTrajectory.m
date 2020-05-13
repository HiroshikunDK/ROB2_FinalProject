%% Setup Kobuki Enviorment - Laptop

setenv('ROS_MASTER_URI','http://192.168.128.129:11311')

setenv('ROS_IP','192.168.1.142')

rosinit('http://192.168.128.129:11311','NodeHost','192.168.1.142');

%% Setup Kobuki Enviorment - HomeStation: 
setenv('ROS_MASTER_URI','http://192.168.1.164:11311')

setenv('ROS_IP','192.168.1.92')

rosinit('http://192.168.1.164:11311','NodeHost','192.168.1.92');

%% Get route
OrgWP = [1.0 1.0; 1.0 4.0;4.0 4.0; 4.0 2.0;6.0 2.0];
WayPoints = OrgWP;
currentPos = [0.0 0.0];
Orientation = 0;

%% Setup

%controller = controllerPurePursuit
%controller.Waypoints = WayPoints
%controller.DesiredLinearVelocity = 0.1;
%controller.MaxAngularVelocity = 0.5;
%controller.LookaheadDistance = 0.5;

%createController(WayPoints,DesiredLinearVelocity,MaximumAngularVelocity, LookAheadDistance)
controller = createController(WayPoints,0.2,0.5,0.2);

robot1 = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");


%% Angle to watch 
%21.83 degres from center 
%0.9283 radinas from center
% if these are under 50 cm, then turtlebot wont fit between them.
%y=asind(2.5);

scandata=get2DScan();

figure
 cart1 = readCartesian(scandata);
    
    x = cart1(:,2); % x-pos
    d = cart1(:,1); % depth
   

    plot(x,d, '.')
    ylim([0 3])
    xlim([-0.7 0.7])
%%
   
   vizRate = rateControl(1/10);

   while (1)
   Drive(1,0);
   disp("command sent")
   waitfor(vizRate);
   %disp(vizRate)
   end  
   
%% Test bugaround 

bugaroundRight();
   
   
   %% Test obstacle
   
   vizRate = rateControl(20);
   startTime = vizRate.TotalElapsedTime;
   something=0;
   while something==0
   %bool=Drive(1,0);
   %disp("command sent")
   waitfor(vizRate);
   if((vizRate.TotalElapsedTime-startTime)>2)
       startTime = startTime +2;
       disp("Interrupt started")
       scandata2 = get2DScan();
       [Right_angle,Right_range,Left_angle,Left_range,isItBlocked] = scanForObstacles(scandata2);
       
       disp(isItBlocked)
   end  
  
   end  
   
%% Run Test function
disp("Test function")
currentpos=[0,0];
Orientation=0;
resetODOM();
PlotWayPoints(OrgWP)
for n = 1 : length(WayPoints)
    [currentPos, Orientation,isAtEnd]=MoveP2P(robot1, currentPos, Orientation, controller, WayPoints(n),OrgWP,0.5);
    if(isAtEnd==0)
        disp("object detected!!!")
        assert(false);
    end
end

%% run Simulation function 

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

    disp("reseODOM called")
    velpub = rospublisher("/mobile_base/commands/reset_odometry");
    msg =rosmessage('std_msgs/Empty');
    send(velpub,msg);
    pause(3);
    disp("reseODOM ended")
    
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


function scandata2 = get2DScan()

    scanlive = rossubscriber("/scan");
    scandata2 = receive(scanlive,10);
    %disp("Scan OK")

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

function [deltaPos] = bugaroundRight()
checktime = 0.3;
deltaTime = 0.3; 
curPos = getCurrentPos()
isClear=0;
turnedcorners=0;
range=0;
range1=10;
impossible=0;

vizRate = rateControl(checktime);
    
    %start process of driving around the obstacle on right side, then
    %switch to left side if failed 
    deltaTime=0.00;
    % 90 degress = 1.5705 rads
    angularRate = -0,5;
    targetAngle = curPos(3)-1.5705;
    
    %driving right
    while(curPos(3) < targetAngle)
        Drive(1,angularRate);
        deltaTime=deltaTime+checktime;
        curPos = getCurrentPos()
        waitfor(vizRate);
    end
    
    %continue to drive towards edge and turnaround corner.
    while((turnedcorners < 2)&&(impossible==0))
    %driving forward
        while(isClear==0)
            if((mod(deltaTime,0.25)==0))
                scandata=get2DScan();
                [range, angle] = getAngleRange(scandata,-1.22);
                [range1, angle1] = getAngleRange(scandata,0);
            end
            if((range > 1)&&(range1 > 1.5))
                isClear=1;
            end
            if((range < 0.75)&&(range1 < 1))
                impossible=1;
            end 
            Drive(1,0);
            deltaTime=deltaTime+checktime;
            waitfor(vizRate);
        end
    
        deltaTime=0.00;
        % 90 degress = 1.5705 rads
        angularRate = 0,5;
        while((deltaTime*angularRate)<1.5705)
            Drive(1,angularRate);
            deltaTime=deltaTime+checktime;
            waitfor(vizRate);
        end
        turnedcorners=turnedcorners+1;
    end
    endPos = getCurrentPos();
    endPos(1) = endpos(1)-startPos(1);
    endPos(2) = endpos(2)-startPos(2);
    deltaPos = endPos;
end

function [range, angle] = getAngleRange(scan,rads)

    
    %Use this section for demo data
    cart1 = readCartesian(scan);
    
    x = cart1(:,2); % x-pos
    d = cart1(:,1); % depth
    
    xslice = [];
    dslice = [];
    
    %plot(x,d, '.'), hold on
    %ylim([0 3])
    %xlim([-0.7 0.7])
    
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
    
    if((lastVisit == -1))
        range = -1;
        angle = 0;
        
        
    
    else
    xslice = x((lastVisit-cnt):lastVisit);
    dslice = d((lastVisit-cnt):lastVisit);
    
    %disp(lastVisit-cnt-10)
    
    mdl = fitlm(xslice,dslice);
    coef=mdl.Coefficients.Estimate;
    
    xP = [(rads-0.2),(rads+0.2)];
    
   % plot(xP, coef(1) + coef(2)*xP, 'r')
        
    range = coef(2)*rads + coef(1);
    angle = rad2deg(atan(coef(2)));
    end


end

function bool = MakeKTurnRight()
    curPos = getCurrentPos();
    targetPos;
    checktime = 0.05;
    vizRate = rateControl(1/checktime);
    
    targetPos=getCurrentPos();
    %turn right 90 degrees, half PI
    targetPos(3)=targetPos(3)-1.571
    while(curPos(3) < targetPos(3))
        
        Drive(1,2);
        curPos = getCurrentPos();
        waitfor(vizRate);
    end
    
    targetPos=getCurrentPos();
    %reverse left 45 degrees, half PI
    targetPos(3)=targetPos(3)-0.7854;
    while(curPos(3) < targetPos(3))
        
        Drive(-1,-2);
        curPos = getCurrentPos();
        waitfor(vizRate);
    end
    
    
    targetPos=getCurrentPos();
    %go ahead in 45 degrees, half PI
    targetPos(3)=targetPos(3)-0.7854;
    while(curPos(3) < targetPos(3))
        
        Drive(1,2);
        curPos = getCurrentPos();
        waitfor(vizRate);
    end
end

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
startTime=vizRate.TotalElapsedTime;
 
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [i, omega] = ControllerObj(robotCurrentPose(1,:));
    release(ControllerObj)
    
    % Get the robot's velocity using controller inputs
    vel = derivative(RobObj, robotCurrentPose(1,:), [i, omega]);
    
    bool=Drive(ControllerObj.DesiredLinearVelocity, vel(3,1));
    
    %calculate the new robot position.
    robotCurrentPose = [initialPos initOrientation] + getCurrentPos(); 
    
    currentTime = vizRate.TotalElapsedTime;
    %interrupt settings
    if((currentTime - startTime)>4)
        startTime=startTime + 4;
        scandata2 = get2DScan();
        [Right_angle,Right_range,Left_angle,Left_range,isItBlocked] = scanForObstacles(scandata2);
    
        if(isItBlocked==1)
            disp("Interrupt Statement triggered")
            endPos = robotCurrentPose(1:2);
            Orientation = robotCurrentPose(3);
            isAtEnd=0;
            return;
        end 
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
    
    endPos = robotCurrentPose(1:2);
    Orientation = robotCurrentPose(3);
    isAtEnd =1;    
end

function bool = PlotWayPoints(WayPoints)
    plot(WayPoints(:,1), WayPoints(:,2),"k--d")
    xlim([0 10])
    ylim([0 10])
end
