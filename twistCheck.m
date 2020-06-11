function [newPose, isblocked] = twistCheck(oriPos, checkAngleDelta, rangeLimit)
    %global  pose saved 
    startPose = getCurrentPos();
    %twist towards the desired angle, twist function ensures correct turn
    Twist(oriPos(3)+checkAngleDelta);
    
    sd = get2DScan();
    
    %check left side if the robots fit through
    [range, angle] = getAngleRange(sd,0.25);
    disp("Twist Test Range:")
    disp(range)   
    if (range<rangeLimit)
        Twist(-checkAngleDelta);
        isblocked=1;
        return
    end
    
    %check right side if the robots fit through
    [range, angle] = getAngleRange(sd,-0.25);
    disp(range)
    if (range<rangeLimit)
        Twist(-checkAngleDelta);
        endPose = getCurrentPos();
        newPose = oriPos + (endPose-startPose);
        isblocked=1;
        return
    end
    
    %The angle is clear and the robot should proced
    endPose = getCurrentPos();
    newPose = oriPos + (endPose-startPose);
    isblocked = 0;    
end

