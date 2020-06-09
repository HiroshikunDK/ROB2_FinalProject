function newPos = calcBugAroundPos(initPos, angle, range)
    
    newPos = initPos;
    disp("StartPostion:")
    disp(newPos)
    newPos(1) = newPos(1)+(cosd(angle))*range; %x coordinate
    newPos(2) = newPos(2)+(sind(angle))*range; %y coordinate 
    disp("targetPostion:")
    disp(newPos)

end

