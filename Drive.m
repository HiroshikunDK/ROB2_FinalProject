function bool = Drive(X,Omega)

    vel = X; %meters per second
    velpub = rospublisher("/mobile_base/commands/velocity");
    velmsg = rosmessage(velpub);

    velmsg.Linear.X = vel;
    velmsg.Angular.Z = Omega/2;
    send(velpub,velmsg);
    bool = 1;

end
