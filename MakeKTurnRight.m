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