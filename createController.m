function controller = createController(WayPoints,DLV,MAV, LAD) 

controller = controllerPurePursuit;
controller.Waypoints = WayPoints;
controller.DesiredLinearVelocity = DLV;
controller.MaxAngularVelocity = MAV;
controller.LookaheadDistance = LAD;

end 
