function [pos,vel,q,acel,giro,t] = capaoPreto()
    RF = "NED";
    fs = 10;
    N_waypoints = 5;
    t = 0:1200/N_waypoints:1200;
    X = 0:150/N_waypoints:150;
    wps = [X' zeros(6,1) zeros(6,1)];
    
    t2 = 0:1200/N_waypoints:1200;
    X2 = 150:-150/N_waypoints:0;
    wps2 = [X2' zeros(6,1) zeros(6,1)];
    
    traj = waypointTrajectory(wps,t,"ReferenceFrame",RF,'AutoBank',true,'AutoPitch',false);
    t = t(1):1/fs:t(end);
    [pos,q,vel,acel,giro] = lookupPose(traj,t);
    
    
    traj = waypointTrajectory(wps2,t2,"ReferenceFrame",RF,'AutoBank',true,'AutoPitch',false);
    t2 = t2(1):1/fs:t2(end);
    [pos2,q2,vel2,acel2,giro2] = lookupPose(traj,t);
    

    tgiro = 60;
    gyroTime = 1:1:tgiro;
    gyroTime = 1200 + gyroTime;

    t2 = 1200 + tgiro + t2;
    eul = quat2eul(q);
    yaw = zeros(tgiro,1);
    
    angle  = eul(end,1);
    for ii=1:1:tgiro
        angle = angle + deg2rad(180/tgiro);
        yaw(ii) = angle;
    end

    qg = quaternion(eul2quat([yaw zeros(tgiro,1) zeros(tgiro,1)]));

    t    = [t gyroTime t2];
    pos  = [pos; pos2(1,1)*ones(tgiro,1) pos2(1,2)*ones(tgiro,1) pos2(1,3)*ones(tgiro,1);pos2];
    q    = [q;qg;q2];    
    vel  = [vel;zeros(tgiro,3);vel2];
    acel = [acel;zeros(tgiro,3);acel2];
    giro = [giro;zeros(tgiro,2) deg2rad(180/tgiro)*ones(tgiro,1);giro2];
end