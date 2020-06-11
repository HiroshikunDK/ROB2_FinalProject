function newPos = calcBugAroundPos(initPos, angle, range)
    
    %get the global position
    newPos = initPos;
    disp("StartPostion:")
    disp(newPos)
    %calc new radian angle with new angle compared to current oriantation
    radOri = det2rad(angle) + newPos(3);
    
    %translate the nre orientation if the radian value
    if (radOri < -3.14159)
        rmd=mod(radOri,-3.14159);
        radOri=3.14159-rmd;
    elseif (radOri > 3.14159)
        rmd=mod(radOri,3.14159);
        radOri=rmd-3.14159;
    else
        disp("Ticket please")
    end
    
    %use trigonometry relations to calculate a x and y value added to the
    %the global position based on orientation and range. 
    newPos(1) = newPos(1)+(cos(radOri))*range; %x coordinate
    newPos(2) = newPos(2)+(sin(radOri))*range; %y coordinate 
    disp("targetPostion:")
    disp(newPos)

end

