function [deltaPos] = bugaroundRight()
checktime = 0.05;
deltaTime = 0.05; 
startPos = getCurrentPos();
isClear=0
turnedcorners=0
range=0;
range1=10;
impossible=0;

vizRate = rateControl(1/checktime);
    
    %start process of driving around the obstacle on right side, then
    %switch to left side if failed 
    deltaTime=0.00;
    % 90 degress = 1.5705 rads
    angularRate = -0,5;
    
    %driving right
    while((deltaTime*angularRate)<1.5705)
        Drive(1,angularRate);
        deltaTime=deltaTime+checktime;
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