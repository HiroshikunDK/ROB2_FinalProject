function pos= getCurrentPos()
odom = rossubscriber("/odom");

odomdata = receive(odom,3);

pose = odomdata.Pose.Pose;

x = pose.Position.X;
y = pose.Position.Y;
quat = pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%theta = rad2deg(angles(1))
Omega = angles(1);
pos= [x y Omega];
end 
