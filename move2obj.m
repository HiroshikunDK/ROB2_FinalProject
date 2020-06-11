function targetPos = move2obj(orgPose,angle1, scandata)
    %angle1 is the angle where the object we want move towards is 
    [range, angle] = getAngleRange(scandata,angle1);
    if(range ~= -1)
        %range -0.8 length is towards the object
        targetPos = calcBugAroundPos(orgPose,angle1, range - 0.8);
    else 
        targetPos = orgPose;
        disp("Error - Invaild Distance")
    end
end
