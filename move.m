function move(X,Omega)

    vel = X; %meters per second
    velpub = rospublisher("/mobile_base/commands/velocity");
    velmsg = rosmessage(velpub);

    velmsg.Linear.X = vel;
    velmsg.Linear.Y = 0;
    velmsg.Linear.Z = 0;
    velmsg.Angular.X = 0;
    velmsg.Angular.Y = 0;
    velmsg.Angular.Z = Omega;
    
    send(velpub,velmsg);
end
