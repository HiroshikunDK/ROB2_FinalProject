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
