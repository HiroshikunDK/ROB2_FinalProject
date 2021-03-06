function [Right_angle,Right_range,Left_angle,Left_range,isItBlocked] = scanForObstacles(scandata)

% find max distance for angle change, can be put outside the function i
% guess
i_counter  = 0;
maxdistance_per_point=zeros(size(16));

range_holder_R=zeros(size(8));
angle_holder_R=zeros(size(8));
range_holder_L=zeros(size(8));
angle_holder_L=zeros(size(8));
for(rad=0.55:-0.1:-0.55)
    deg = (rad*(180/3.14));
    calc_point = deg;
    %180-(90+deg)
    i_counter = i_counter+1;
    %maxdistance_per_point(i_counter) = (1.0/(sind(calc_point)))*(sind(90));
end
maxDistVar = 1;

%check each range fits with estimate and keep the ones that dont
i_counter = 0;
fail_R = 0;
fail_L = 0;

%disp("Range left: ")

for(rad=0.55:-0.1:0)
    [range, angle]=getAngleRange(scandata,rad);
    i_counter = i_counter+1;
    %disp(range)
    if((range < maxDistVar) && (range > 0))
        fail_R = 1;
        range_holder_R(i_counter) = range;
        angle_holder_R(i_counter) = angle;
    end
end

i_counter = 0; %last half of maxdistance_per_point
%disp("Range right") 
for(rad=-0.55:+0.1:0)
    [range, angle]=getAngleRange(scandata,rad);
    i_counter = i_counter+1;
    %disp(range)
   if((range < maxDistVar) && (range > 0))
        fail_R = 1;
        range_holder_L(i_counter) = range;
        angle_holder_L(i_counter) = angle;
    end
end
% set is blocked
if((fail_R || fail_L) == 1)
isItBlocked = 1;
Right_range = range_holder_R;
Right_angle = angle_holder_R;
Left_range = range_holder_L;
Left_angle = angle_holder_L;
else
isItBlocked = 0; 
Right_range = 0;
Right_angle = 0;
Left_range = 0;
Left_angle = 0;
end
%disp(isItBlocked)
%should return the approx length of the object, so we know where to go past
%it 
%width = 0;
% should be 1 if obstacle is detected, within the threshold. 
end
