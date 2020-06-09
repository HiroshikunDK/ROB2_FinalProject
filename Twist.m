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

