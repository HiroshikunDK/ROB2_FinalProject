function scandata = get2DScan()

    scanlive = rossubscriber("/scan");
    scandata = receive(scanlive,10);
    %disp("Scan OK")

end 
