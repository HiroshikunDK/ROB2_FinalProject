function bool = PlotWayPoints(WayPoints)
    plot(WayPoints(:,1), WayPoints(:,2),"k--d")
    xlim([0 10])
    ylim([0 10])
end
