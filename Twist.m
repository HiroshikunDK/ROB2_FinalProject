function Twist(radian)
    startPos = getCurrentPos()
    radOri=startPos(3)+radian;
    if(radOri<-3.14159)
        rmd=mod(radOri,-3.14159);
        endOri=rmd+3.14159;     
    elseif(radOri>3.14159)
        rmd=mod(radOri,3.14159);
        endOri=rmd-3.14159;
    else
        
    end
    delta=1;    
    curOri = startPos(3)
    while(((delta<0.01)&&(delta>-0.01))==0)
        
        %calculate turning distance turning left(positive radian value)
        if(curOri>endOri)
            tempLeft=(3.14159+endOri)+(3.14159-curOri);    
        else
            tempLeft= endOri-curOri;
        end
        
        %calculate turning distance turning right(negative radian value)
        if(curOri<endOri)
            tempRight=-(3.14159-endOri)-(3.14159-curOri);    
        else
            tempRight= curOri-endOri;
        end
        
        %this find the shortest way to turn towards target
        if((tempRight*-1)<tempLeft)
            delta = tempRight;
        else
            delta = tempLeft;
        end
        
        %sends the twist message 
        velpub = rospublisher("/mobile_base/commands/velocity");
        velmsg = rosmessage(velpub); 
        velmsg.Angular.Z = delta;
        send(velpub,velmsg);
        
        startPos = getCurrentPos();
        curOri = startPos(3);
    end
end

