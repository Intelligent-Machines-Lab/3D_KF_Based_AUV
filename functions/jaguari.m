function [pos,vel,q,acel,giro,t] = jaguari()
    RF = "NED";
    fs = 10;
    N_waypoints = 5;
    t = 0:1200/N_waypoints:1200;
    X = 0:200/N_waypoints:200;
    wps = [X' zeros(6,1) zeros(6,1)];
    
    t2 = 0:1200/N_waypoints:1200;
    X2 = 200:-200/N_waypoints:0;
    wps2 = [X2' zeros(6,1) zeros(6,1)];
    
    traj = waypointTrajectory(wps,t,"ReferenceFrame",RF,'AutoBank',true,'AutoPitch',false);
    t = t(1):1/fs:t(end);
    [pos,q,vel,acel,giro] = lookupPose(traj,t);
    
    
    traj = waypointTrajectory(wps2,t2,"ReferenceFrame",RF,'AutoBank',true,'AutoPitch',false);
    t2 = t2(1):1/fs:t2(end);
    [pos2,q2,vel2,acel2,giro2] = lookupPose(traj,t);
    
    N_giro =  60;
    gyroTime = 1:1:N_giro;
    gyroTime = 1200 + gyroTime;

    t2 = 1200+N_giro+ t2;
    eul = quat2eul(q);
    yaw = zeros(N_giro,1);
    
    angle  = eul(end,1);
    for ii=1:1:N_giro
        angle = angle + deg2rad(180/N_giro);
        yaw(ii) = angle;
    end

    qg = quaternion(eul2quat([yaw zeros(N_giro,1) zeros(N_giro,1)]));

    t    = [t gyroTime t2];
    pos  = [pos; pos(end,1)*ones(N_giro,1) pos(end,2)*ones(N_giro,1) pos(end,3)*ones(N_giro,1);pos2];
    q    = [q;qg;q2];    
    vel  = [vel;zeros(N_giro,3);vel2];
    acel = [acel;zeros(N_giro,3);acel2];
    giro = [giro;zeros(N_giro,2) deg2rad(180/N_giro)*ones(N_giro,1);giro2];
end