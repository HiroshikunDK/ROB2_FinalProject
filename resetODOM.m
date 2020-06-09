function resetODOM()

    disp("reseODOM called")
    velpub = rospublisher("/mobile_base/commands/reset_odometry");
    msg =rosmessage('std_msgs/Empty');
    send(velpub,msg);
    pause(3);
    disp("reseODOM ended")
    
    bool = 1;

end
