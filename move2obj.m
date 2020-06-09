function targetPos = move2obj(orgPose,angle1, scandata)
    [range, angle] = getAngleRange(scandata,angle1);
    if(range ~= -1)
        targetPos = calcBugAroundPos(orgPose,angle1, range - 0.8);
    else 
        targetPos = orgPose;
        disp("Error - Invaild Distance")
    end
end
