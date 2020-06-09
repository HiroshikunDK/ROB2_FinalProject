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

