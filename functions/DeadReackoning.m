function DeadReackoning()
clc;close all;clear all;
addpath('./functions')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Load all sensors data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load("MAIN.mat")
path = "";
pasta = MAIN.folder;

load(path+pasta+"imu_data.mat")
load(path+pasta+"dvl_data.mat")
load(path+pasta+"mag_data.mat")
load(path+pasta+"depth_data.mat")
load(path+pasta+"p881L_data.mat")
load(path+pasta+"mag_cal.mat")
% load(path+pasta+"rpy_data.mat")
load(path+pasta+"gps_data.mat")



pOffset = 3*pi/2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initial and final time of simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t0 = imu_data.Time(size(1,1));
tf = imu_data.Time(size(imu_data,1));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Getting the size and the period of each array data
% IMU
% DVL 
% MAG 
% GPS 
% DEPTH
% profiling 881L 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
imu_size = size(imu_data,1);
imu_dt   = (tf - t0)/imu_size;

dvl_size = size(dvl_data,1);
dvl_dt   = (tf - t0)/dvl_size;

mag_size = size(mag_data,1);
mag_dt   = (tf - t0)/mag_size;

depth_size = size(depth_data,1);
depth_dt   = (tf - t0)/depth_size;

p881L_size = size(p881L_data,1);
p881L_dt   = (tf - t0)/p881L_size;

% rpy_size = size(rpy_data,1);
% rpy_dt   = (tf - t0)/rpy_size;

gps_size = size(gps_data,1);
gps_dt   = (tf - t0)/gps_size;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Flags to indicate that have a new sensor menssage
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
imu_newData   = false;
dvl_newData   = false;
mag_newData   = false;
depth_newData = false;
gps_newData   = false;
p881L_newData = false;
rpy_newData   = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Flags to indicate that have a new sensor menssage
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
imu_find   = 1;
dvl_find   = 1;
mag_find   = 1;
depth_find = 1;
gps_find   = 1;
p881L_find = 1;
rpy_find   = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialize body variables (contain the current sensors value)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
body_ax = 0; body_ay = 0; body_az = 0;

body_roll = 0; body_pitch = 0; body_yaw = 0;

body_gx = 0; body_gy = 0; body_gz = 0;

body_vx = 0; body_vy = 0; body_vz = 0;

body_pointX = 0; body_pointY = 0; body_pointZ = 0;

mag_mx = 0; mag_my = 0; mag_mz = 0;

gps_px = 0; gps_py = 0;

depth_pz = 0;

p881L.intensities  = [];
p881L.currentAngle = 0;

global_posX = 0; global_posY = 0; global_posZ = 0;
global_pointX = 0; global_pointY = 0; global_pointZ = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Calibration variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
IMU_BIAS  = [0 0 0 0 0 0]';
%MAG_SCALE = [1 1 1];%MAG_CALIBRATION.SCALE;
%MAG_BIAS  = [0 0 0]%MAG_CALIBRATION.BIAS;
MAG_SCALE = MAG_CALIBRATION.SCALE;
MAG_BIAS  = MAG_CALIBRATION.BIAS;
MAG_DEC   = MAG_CALIBRATION.DEC;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Main Loop code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
jj = seconds(t0);

% structure to store all dead reackoning data in a csv file to plot
DRCSV = zeros(dvl_size,13);
dt    = jj;
dt_imu = jj;

imu_idx = 1;
imu_arrayTime = seconds(imu_data.Time);

dvl_idx = 1;
dvl_arrayTime = seconds(dvl_data.Time);

mag_idx = 1;
mag_arrayTime = seconds(mag_data.Time);


depth_idx = 1;
depth_arrayTime = seconds(depth_data.Time);

p881L_idx = 1;
p881L_arrayTime = seconds(p881L_data.Time);

% rpy_idx = 1;
% rpy_arrayTime = seconds(rpy_data.Time);

gps_idx = 1;
gps_arrayTime = seconds(gps_data.Time);

dt_print_inc = 1000*seconds(p881L_dt/2);
dt_print = jj;

is_valid = 0;
alpha = 0.9;
angle_filtered = [0 0 0]';
accel_angle = [0 0 0]';
gyro_angle = [0 0 0]';
first_run = true;

for ii = seconds(t0) : seconds(p881L_dt/2) : seconds(tf)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % update the time counter ii getting the current time value
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Get new sensor data
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     data = imu_arrayTime(imu_idx);
     if ii > data
        imu_idx = imu_idx + 1;
        if imu_idx > imu_size
            imu_idx = imu_size;
        end
        imu_newData = true;
     end
     
     if imu_newData
        body_ax = imu_data.IMU_TABLE(imu_idx,1);
        body_ay = imu_data.IMU_TABLE(imu_idx,2);
        body_az = imu_data.IMU_TABLE(imu_idx,3);
        
        body_gx = imu_data.IMU_TABLE(imu_idx,4);
        body_gy = imu_data.IMU_TABLE(imu_idx,5);
        body_gz = imu_data.IMU_TABLE(imu_idx,6);
     end
     
     % data = rpy_arrayTime(rpy_idx);
     % if ii > data
     %    rpy_idx = rpy_idx + 1;
     %    if rpy_idx > rpy_size
     %        rpy_idx = rpy_size;
     %    end
     %    rpy_newData = true;
     % end
     % 
     % if rpy_newData
     %    body_roll  = rpy_data.RPY_TABLE(rpy_idx,3);
     %    body_pitch = rpy_data.RPY_TABLE(rpy_idx,2);
     %    body_yaw   = rpy_data.RPY_TABLE(rpy_idx,1);
     % 
     %    rpy_newData = false;
     % end
     
     if length(dvl_arrayTime) ~= 0
        data = dvl_arrayTime(dvl_idx);
     else
         data = 9999999999;
     end

     %data = dvl_arrayTime(dvl_idx);
     if ii > data
        dvl_idx = dvl_idx + 1;
        
        if dvl_idx > dvl_size
            dvl_idx = dvl_size;
        end
        dvl_newData = true;
     end

     if dvl_newData
        
        is_valid = dvl_data.DVL_TABLE(dvl_idx,4);        
        if is_valid
            body_vx = dvl_data.DVL_TABLE(dvl_idx,1);
            body_vy = dvl_data.DVL_TABLE(dvl_idx,2);
            body_vz = dvl_data.DVL_TABLE(dvl_idx,3);
        end
     end
     
          if length(gps_arrayTime) ~= 0
        data = gps_arrayTime(gps_idx);
     else
         data = 9999999999;
     end

     if ii > data
        gps_idx = gps_idx + 1;
        
        if gps_idx > gps_size
            gps_idx = gps_size;
        end
        gps_newData = true;
     end

     if gps_newData
        gps_px = gps_data.gps(gps_idx,1) ;
        gps_py = gps_data.gps(gps_idx,2) ;
        gps_newData = false;
     end

     data = mag_arrayTime(mag_idx);
     if ii > data
        mag_idx = mag_idx + 1;
        if mag_idx > mag_size
            mag_idx = mag_size;
        end
        mag_newData = true;
     end
     
     
     if mag_newData
        
        mag_mx = mag_data.MAG_TABLE(mag_idx,1);
        mag_my = mag_data.MAG_TABLE(mag_idx,2);
        mag_mz = mag_data.MAG_TABLE(mag_idx,3);
     end
     
     data = depth_arrayTime(depth_idx);
     if ii > data
        depth_idx = depth_idx + 1;
        if depth_idx > depth_size
            depth_idx = depth_size;
        end
        depth_newData = true;
     end

     if depth_newData
        depth_newData = false;
        depth_pz = depth_data.PRESSURE_TABLE(depth_idx,1);
     end
           
     data = p881L_arrayTime(p881L_idx);
     if ii > data
        p881L_idx = p881L_idx + 1;
        if p881L_idx > p881L_size
            p881L_idx = p881L_size;
        end
        p881L_newData = true;
     end

     if p881L_newData
        p881L_newData = false;
        [val, idx] = max(p881L_data.Intensities(p881L_idx,1:500));

        range = double(p881L_data.Range(p881L_idx,1)/1000);
        p881L.intensities  = double(range*idx/500);
        p881L.currentAngle = p881L_data.Angle(p881L_idx,1) + pOffset;
        

        body_pointX = 0;
        body_pointY =     p881L.intensities*cos(p881L.currentAngle);
        body_pointZ =     p881L.intensities*sin(p881L.currentAngle);
     end
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Treat the new sensor data
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if imu_newData
        Timu = (ii - dt_imu);
        dt_imu = ii;

        body_ax =  body_ax -IMU_BIAS(1);
        body_ay =  body_ay -IMU_BIAS(2);
        body_az =  body_az -IMU_BIAS(3);
        
        body_gx =  body_gx -IMU_BIAS(4);
        body_gy =  body_gy -IMU_BIAS(5);
        body_gz =  body_gz -IMU_BIAS(6);
        
        
        g = norm([body_ax body_ay body_az]);
        ax = body_ax/g;
        ay = body_ay/g;
        az = body_az/g;

        accel_angle(1) = atan(ay/az);
        accel_angle(2) = asin(-ax);
        %accel_angle(2) = atan(ax/(g*sqrt(ay*ay+az*az)));
        
        sr = sin(angle_filtered(1));
        cr = cos(angle_filtered(1));
        sp = sin(angle_filtered(2));
        cp = cos(angle_filtered(2));

        Rw = (1/cp)*[cp sr*sp -cr*sp;0 cr*cp sr*cp;0 -sr cr];
        %Rw = eye(3);
        gyro_angle  = Rw*[body_gx body_gy body_gz]'*Timu;
        imu_newData = false;
    end
    
    if mag_newData 
        mag_mx =  ( mag_mx - MAG_BIAS(1))/MAG_SCALE(1);
        mag_my =  ( mag_my - MAG_BIAS(2))/MAG_SCALE(2);
        mag_mz =  ( mag_mz - MAG_BIAS(3))/MAG_SCALE(3);
        
         n_m = norm([mag_mx mag_my mag_mz]);
         
         mag_mx = mag_mx/n_m;
         mag_my = mag_my/n_m;
         mag_mz = mag_mz/n_m;

        if first_run            
            accel_angle(3) = MAG_DEC + getHeading(mag_mx,mag_my);
            first_run = false;
        end
        mm = getRotation(0,angle_filtered(2),angle_filtered(1))*[mag_mx mag_my mag_mz]';
        n_mm = norm(mm);
        mm = mm/n_mm;
        accel_angle(3) =  MAG_DEC + getHeading(mm(1),mm(2));
        %accel_angle(3) =  MAG_DEC + getHeading(mag_mx,mag_my);
        mag_newData = false;
    end
 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dead Reckoning
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if dvl_newData
        T = (ii - dt);
        dt = ii;
       
        angle_filtered = alpha*(angle_filtered + gyro_angle) + (1-alpha)*(accel_angle);
        roll  = angle_filtered(1);
        pitch = angle_filtered(2);
        yaw   = angle_filtered(3);
        %roll = 0;
        %pitch = 0;
        %yaw = MAG_DEC + getHeading(mag_mx,mag_my);

        vec_vel = getRotation(yaw,pitch,roll)*[body_vx body_vy body_vz]';
        
        global_vx = vec_vel(1);
        global_vy = vec_vel(2);
        global_vz = vec_vel(3);
    if MAIN.isGPS
        global_posX = gps_px ; 
        global_posY = gps_py;
        global_posZ = -depth_pz;
    else
        global_posX = global_posX + global_vx*T; 
        global_posY = global_posY + global_vy*T;
        %global_posZ = global_posZ + global_vz*T;
        global_posZ = -depth_pz;
        %disp('aqu')
    end

        global_point = getRotation(yaw,pitch,roll)*[body_pointX -body_pointY -body_pointZ]';
         
        global_pointX = global_posX + global_point(1); 
        global_pointY = global_posY + global_point(2);
        global_pointZ = global_posZ + global_point(3);

        dvl_newData = false;
       
        DRCSV(dvl_idx,:) = [(ii),global_posX,global_posY,global_posZ,...
                   global_vx,global_vy,global_vz,...
                   roll,pitch,yaw,...
                   global_pointX,global_pointY,global_pointZ];
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % update the time counter jj getting the past value
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    jj = ii;
    
    if ii > dt_print + dt_print_inc 
        dt_print = ii;
        disp( "iteração: " +(ii)+ " de "+seconds(tf));
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% save data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save(path+pasta+'DRCSV.mat','DRCSV');
%csvwrite('Dead_Reckoning.csv', DRCSV);

end