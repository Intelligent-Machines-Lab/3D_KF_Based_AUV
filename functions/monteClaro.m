function [pos,vel,q,acel,giro,t] = monteClaro()
    RF = "NED";
    fs = 10;
    N_waypoints = 5;

    t   = 0:7200/N_waypoints:7200;
    X   = 0:1200/N_waypoints:1200;
    wps = [X' zeros(6,1) zeros(6,1)];
    traj = waypointTrajectory(wps,t,"ReferenceFrame",RF,'AutoBank',true,'AutoPitch',false);
    t = t(1):1/fs:t(end);
    [pos,q,vel,acel,giro] = lookupPose(traj,t);
end