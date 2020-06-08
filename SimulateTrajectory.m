%% Setup Kobuki Enviorment - Laptop

setenv('ROS_MASTER_URI','http://192.168.128.129:11311')

setenv('ROS_IP','192.168.100.105')

rosinit('http://192.168.128.129:11311','NodeHost','192.168.100.105');

%% Setup Kobuki Enviorment - HomeStation: 
setenv('ROS_MASTER_URI','http://192.168.1.164:11311')

setenv('ROS_IP','192.168.1.92')

rosinit('http://192.168.1.164:11311','NodeHost','192.168.1.92');

%% Get route
OrgWP = [1.0 1.0; 2.0 4.0;8.0 4.0;8.0 2.0;4.0 4.0];
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
controller = createController(WayPoints,0.25,0.4,0.35);
controllerCrawl = createController(WayPoints,0.15,0.5,0.25);

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
    ylim([0 8])
    xlim([-1.5 1.5])
    
billede = rossubscriber("/camera/rgb/image_raw");

img = receive(billede);

t= readImage(img);
imtool(t); %opens inspector mode on image

imwrite(t,'C:\Users\thoma\Documents\AU_IKT\_ROB2\Final_project\test.jpg')
%%
   
   vizRate = rateControl(1/4);

   while(1)
   
   %bool=move(1.0,0);
   
   %
   %executed every 4. second
    velpub = rospublisher("/mobile_base/commands/velocity");
    velmsg = rosmessage(velpub)

    velmsg.Linear.X = 1;
    velmsg.Linear.Y = 0;
    velmsg.Linear.Z = 0;
    velmsg.Angular.X = 0;
    velmsg.Angular.Y = 0;
    velmsg.Angular.Z = 0;
    
    pause(1)
    
    send(velpub,velmsg);
   
   
   disp("command sent")
   waitfor(vizRate);
   %disp(vizRate)
   
   end  
   
%% Test bugaround 

%scandata = get2DScan();
%[Right_angle,Right_range,Left_angle,Left_range,isItBlocked] = scanForObstacles(scandata);
%disp("isItBlocked:")
%disp(isItBlocked)   

globalpos=[0,0,0];

sdtest=get2DScan();
deltaPos1 = move2obj(globalpos,0.7,sdtest)
newcontroller = createController([deltaPos1(1:2)],0.15,0.5,0.25);
[globalpos,isAtEnd]=MoveP2P(robot1, globalpos, newcontroller, deltaPos1(1:2),0.1);
disp("Done")

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
    [currentPos, Orientation,isAtEnd]=MoveP2P(robot1, currentPos, Orientation, controller, WayPoints(n),OrgWP,0.1);
    if(isAtEnd==0)
        bugaround(robot1, controllerCrawl, OrgWP, 0.1)
        disp("Bugaround ended!!!!")  
    end
    disp("Waypoint reached: ")
    disp(n)
end

%% run Simulation function 

%resetODOM();

%frameSize =MakePlot(WayPoints);

%distanceToGoal = norm(currentPos(1,1:2) - robotGoal(:));
goalRadius = 0.2;

for n = 1 : length(WayPoints)
    [currentPos, Orientation,isAtEnd]=MoveP2PSim(robot1, currentPos, Orientation, controller, WayPoints(n),OrgWP,0.1);
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


%% test image analysis

%reset all robot info
globalPos = [0, 0, -1];

deltaPos = calcBugAroundPos(globalPos,-90, 0.7);

%globalPos = bugaround(robot1, 0.1, globalPos)
%disp("Bugaround ended!!!!")
%Twist(3.141/2);

%% Model Trajectory functions
function newPose = bugaround(RobObj, SampleTime , initPos)
    %scan obstacle

    disp("Move along citizen ")
    sd=get2DScan();
    [range, angle] = getAngleRange(sd,0);
    disp([range, angle])
    
    deltaPos = calcBugAroundPos(initPos,-90+angle, 0.7);
    currentPose=initPos;
    Twist(-3.14159/2); 
    
    disp("CurrentPost:")
    disp(deltaPos)
    tempControl = createController([deltaPos(1:2)],0.15,0.5,0.25);
    [currentPose, isAtEnd]=MoveP2P(RobObj, currentPose, tempControl, deltaPos(1:2),SampleTime);
    
    %reset the variable
    isAtEnd=1;
    turnedCorners=0;
    
    while((turnedCorners ~= 2)&&(turnedCorners ~= -2))
        disp("Starting to check turns:")
        
        %reset the variable 
        isBlocked=1;
        
        while((isBlocked==1)&&(isAtEnd==1))
            %turn left and scan for object, will place itself along the object.
            [currentPose, isblocked] = twistCheck(pos(3), 3.14159/2, 1.5);
            disp("Postion angle checked")
            disp(pos)
            %move forward along the along the object
            deltaPos = calcBugAroundPos(0, 1); %drive forward 
            tempControl = createController([deltaPos(1:2)],0.15,0.5,0.25);
            [currentPos, Orientation,isAtEnd]=MoveP2P(RobObj, currentPose, tempControl, deltaPos(1:2),SampleTime);
            if(isBlocked==0)
                disp("Corner Turned")
                turnedCorners=turnedCorners+1
            end
            
        end 
    end 
  
end

%end position should be decided.
function newPos = calcBugAroundPos(initPos, angle, range)
    
    newPos = initPos;
    disp("StartPostion:")
    disp(newPos)
    newPos(1) = newPos(1)+(cosd(angle))*range; %x coordinate
    newPos(2) = newPos(2)+(sind(angle))*range; %y coordinate 
    disp("targetPostion:")
    disp(newPos)

end

function [newPose, isblocked] = twistCheck(oriPos, checkAngleDelta, rangeLimit)
    startPose = getCurrentPos();
    Twist(oriPos(3)+checkAngleDelta);
    sd = get2DScan();
    [range, angle] = getAngleRange(sd,0.25);
    disp("Twist Test Range:")
    disp(range)   
    if (range<rangeLimit)
        Twist(-checkAngleDelta);
        isblocked=1;
        return
    end
    
    [range, angle] = getAngleRange(sd,-0.25);
    disp(range)
    if (range<rangeLimit)
        Twist(-checkAngleDelta);
        endPose = getCurrentPos();
        newPose = oriPos + (endPose-startPose);
        isblocked=1;
        return
    end
    endPose = getCurrentPos();
    newPose = oriPos + (endPose-startPose);
    isblocked = 0;    
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


function bool = PlotWayPoints(WayPoints)
    plot(WayPoints(:,1), WayPoints(:,2),"k--d")
    xlim([0 10])
    ylim([0 10])
end



function Twist(radian)
    startPos = getCurrentPos()
    if(startPos(3)+radian>3.14159)
        endOri=(startPos(3)+radian)-(3.14159*2);     
    elseif(startPos(3)+radian<-3.14159)
        endOri=(startPos(3)+radian)+(3.14159*2);
    else
        endOri=startPos(3)+radian;
    end
        
    curOri = startPos(3)
    while((((endOri-curOri)<0.01)&&((endOri-curOri)>-0.01))==0)
        delta = endOri-curOri;

        velpub = rospublisher("/mobile_base/commands/velocity");
        velmsg = rosmessage(velpub);
        if(delta<0.1)
            velmsg.Angular.Z = delta;
        else
            velmsg.Angular.Z = delta;
        end
        send(velpub,velmsg);
        startPos = getCurrentPos();
        curOri = startPos(3);
    end
end

function resetODOM()

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

function move(X,Omega)

    vel = X; %meters per second
    velpub = rospublisher("/mobile_base/commands/velocity");
    velmsg = rosmessage(velpub);

    velmsg.Linear.X = vel;
    velmsg.Angular.Z = Omega;
    send(velpub,velmsg);

end

function targetPos = move2obj(orgPose,angle1, scandata)
    [range, angle] = getAngleRange(scandata,angle1);
    if(range ~= -1)
        targetPos = calcBugAroundPos(orgPose,angle1, range - 0.8);
    else 
        targetPos = orgPose;
        disp("Error - Invaild Distance")
    end
end

function controller = createController(WayPoints,DLV,MAV, LAD) 
controller = controllerPurePursuit;
controller.Waypoints = WayPoints;
controller.DesiredLinearVelocity = DLV;
controller.MaxAngularVelocity = MAV;
controller.LookaheadDistance = LAD;

end 

