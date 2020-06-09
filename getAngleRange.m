function [range, angle] = getAngleRange(scan,rads)

    
    %Use this section for demo data
    cart1 = readCartesian(scan);
    
    x = cart1(:,2); % x-pos
    d = cart1(:,1); % depth
    
    xslice = [];
    dslice = [];
    
    %plot(x,d, '.'), hold on
    %ylim([0 3])
    %xlim([-0.7 0.7])
    
    lastVisit = -1;
    cnt=0;
    
    for j = 1:1:length(x)

        if (x(j)>(rads-0.005))&&(x(j)<(rads+0.005))
            %fjern comment for at se hvilke xværdier er brugt. 
            %disp(x(j))
            lastVisit = j;
            cnt = cnt + 1;
        end
    end
    
    %disp("lastVisit: ")
    %disp(lastVisit)
    %disp("count: ")
    %disp(cnt)    
    
    if((lastVisit == -1))
        range = -1;
        angle = 0;
        
        
    
    else
    xslice = x((lastVisit-cnt):lastVisit);
    dslice = d((lastVisit-cnt):lastVisit);
    
    %disp(lastVisit-cnt-10)
    
    mdl = fitlm(xslice,dslice);
    coef=mdl.Coefficients.Estimate;
    
    xP = [(rads-0.2),(rads+0.2)];
    
   % plot(xP, coef(1) + coef(2)*xP, 'r')
        
    range = coef(2)*rads + coef(1);
    angle = rad2deg(atan(coef(2)));
    end


end
