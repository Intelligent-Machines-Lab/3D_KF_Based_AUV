%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Author:Juan R. B. F. S.
%3D Kalman Filter Based
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%function KF_BASED()
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
%load(path+pasta+"rpy_data.mat")
load(path+pasta+"gps_data.mat")
load(path+pasta+"PARAMS.mat")


if MAIN.isGPS == 0
    gps_data = [];
    gps_data.Time = [];
end

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
KF_newData    = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialize body variables (contain the current sensors value)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
body_ax = 0; body_ay = 0; body_az = 0;

body_roll = 0; body_pitch = 0; body_yaw = 0;

body_gx = 0; body_gy = 0; body_gz = 0;

body_vx = 0; body_vy = 0; body_vz = 0;

body_pointX = 0; body_pointY = 0; body_pointZ = 0;

mag_mx = 0; mag_my = 0; mag_mz = 0;

depth_pz = 0;

gps_px = 0; gps_py = 0;

p881L.intensities  = [];
p881L.currentAngle = 0;

global_posX = 0; global_posY = 0; global_posZ = 0;
global_pointX = 0; global_pointY = 0; global_pointZ = 0;

global_point_KF = [0 0 0]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Calibration variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

BIAS  = [0 0 0 0 0 0]';
MAG_SCALE = MAG_CALIBRATION.SCALE;
MAG_BIAS  = MAG_CALIBRATION.BIAS;
MAG_DEC   = MAG_CALIBRATION.DEC;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%KF-BASED variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initial conditions
px0 = 0;
py0 = 0;
pz0 = 0;
vx0 = 0;
vy0 = 0;
vz0 = 0;
psi0   = 0;
roll0  = 0;
pitch0 = 0;
bax0 = 0;
bay0 = 0;
baz0 = 0;
bwx0 = 0;
bwy0 = 0;
bwz0 = 0;
 
X_INS = [vx0;vy0;vz0;px0;py0;pz0;roll0;pitch0;psi0];
BIAS  = [bax0;bay0;baz0;bwx0;bwy0;bwz0];
X_E   = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];

%vx, vy, vz, px, py, pz, roll, pitch, yaw, bax, bay, baz, bwx, bwy, bwz
%P0 = diag([1 1 1 10 10 10 10*pi/180 10*pi/180 10*pi/180 1 1 1 0.1 0.1 0.1]);
P0 = diag(PARAMS.P);
Pk = P0*P0';
Pk_1 = Pk;

% ax, ay, az, wx, wy wz
%Q  = diag([0.1 0.1 0.1 0.1 0.1 0.1]);
%Q  = diag([1 1 1 1 1 1]);
%Q  = diag([0.01 0.01 0.01 0.01 0.01 0.01]);
Q = diag(PARAMS.Q);
Q = Q*Q';

% vx,vy,vz, px, py, pz, roll, pitch, yaw
%R  = diag([ 0.02 0.02 0.02 0.1 5*pi/180 5*pi/180 5*pi/180]); 
%R  = diag([ 0.02 0.02 0.02 0.1 5*pi/180 5*pi/180 5*pi/180]);
%R  = diag([ 0.1 0.1 0.1 2 2 0.1 5*pi/180 5*pi/180 5*pi/180]);
R = diag(PARAMS.R);
R = R*R';

%
betta = diag(PARAMS.betta);

C = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Main Loop code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
jj = seconds(t0);

% structure to store all KF-BASED data in a csv file to plot
KFCSV      = zeros(p881L_size,15);
KFCSV_XE   = zeros(p881L_size,16);
KFCSV_XED  = zeros(p881L_size,16);
mat_Pk     = zeros(p881L_size,225);
mat_Pk_1   = zeros(p881L_size,225);
mat_Ik     = zeros(p881L_size,10);
mat_STD_Ik = zeros(p881L_size,10);

dt_KF = jj;
dt_KF_MAG = jj;
first_run = true;

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

X_EP = [0 0 0 0 0 0 0 0 0]';
IK   = [0 0 0 0 0 0 0 0 0]';
STD_IK  = [0 0 0 0 0 0 0 0 0]';

angle_filtered = [0 0 0]';
accel_angle = [0 0 0]';
gyro_angle = [0 0 0]';

lpf = [0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%KF mag calibration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sq = 10000;
Q_MAG   = sq*diag([1 1 1 1 1 1 1 1 1]);% system noise variance
Q_MAG = Q_MAG*Q_MAG';

sr = 1;
R_MAG      = sr*diag([1 1 1 1 1 1]);% observation noise variance
R_MAG      = R_MAG*R_MAG';

sp = 10000;
P_MAG = sp*diag([1 1 1 1 1 1 1 1 1]);
P0_MAG = P_MAG;

%MAG_BIAS  = [ 2000 500];
%BIAS = [900 900 300 ];
%SCALE = [2000 3000 1000];
%SCALE = [1600 2700 950];

M_BIAS  = MAG_BIAS';
M_SCALE = MAG_SCALE';
MAG_NORTH = [sin(MAG_DEC) cos(MAG_DEC)   0]';

mx = (mag_data.MAG_TABLE(1,1) - M_BIAS(1))*M_SCALE(1);
my = (mag_data.MAG_TABLE(1,2) - M_BIAS(2))*M_SCALE(2);
mz = (mag_data.MAG_TABLE(1,3) - M_BIAS(3))*M_SCALE(3);
n_m = norm([mx my mz]);

MAG = [mx my mz]/n_m ;
X_MAG = [MAG M_BIAS M_SCALE]';

%X_MAG = [M_BIAS M_SCALE]';
magCount = 1;

%mag_size = mag_size
MAGCSV = zeros(mag_size,31);
REF_MAGCSV = zeros(mag_size,13);
betta = 0.02;
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
     % 
     % if rpy_newData
     %    body_roll  = rpy_data.RPY_TABLE(rpy_idx,3);
     %    body_pitch = rpy_data.RPY_TABLE(rpy_idx,2);
     %    body_yaw   = rpy_data.RPY_TABLE(rpy_idx,1);
     % 
     %    %rpy_newData = false;
     % end

     if length(dvl_arrayTime) ~= 0
        data = dvl_arrayTime(dvl_idx);
     else
         data = 9999999999;
     end

     if ii > data
        dvl_idx = dvl_idx + 1;
        
        if dvl_idx > dvl_size
            dvl_idx = dvl_size;
        end
        dvl_newData = true;
     end

     if dvl_newData
        body_vx = dvl_data.DVL_TABLE(dvl_idx,1);
        body_vy = dvl_data.DVL_TABLE(dvl_idx,2);
        body_vz = dvl_data.DVL_TABLE(dvl_idx,3);
        is_valid = dvl_data.DVL_TABLE(dvl_idx,4);
        %if ~(is_valid == 1)
        %    dvl_newData = false;
        %end
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
        gps_px = gps_data.gps(gps_idx,1);
        gps_py = gps_data.gps(gps_idx,2);
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

     if mag_newData && 1
        T_KF_MAG = (ii - dt_KF_MAG);
        dt_KF_MAG = ii;
        
        wx = body_gx - BIAS(4);
        wy = body_gy - BIAS(5);
        wz = body_gz - BIAS(6);

        S = [ 0 -wz wy;wz 0 -wx;-wy wx 0];
        J = [-S S zeros(3);zeros(6,9)];

        T_KF_MAG = T_KF_MAG;
        Ad = expm(J*T_KF_MAG);
        
        X_MAG = Ad*X_MAG;

        
        P_MAG = Ad*P_MAG*Ad' + T_KF_MAG*T_KF_MAG*Q_MAG;% + betta*P0_MAG;
        
        mx = (mag_mx -M_BIAS(1))*M_SCALE(1);
        my = (mag_my -M_BIAS(2))*M_SCALE(2);
        

        r   = 0;
        p   = 0;
        y   = getHeading(mx,my);
        
        mo = getRotation(y,p,r)'*MAG_NORTH;
        
        H_MAG = [eye(3) zeros(3) zeros(3);
                 zeros(3) eye(3) diag(mo)];
        %
        S_MAG = H_MAG*P_MAG*H_MAG' + R_MAG;
        K_MAG = P_MAG*H_MAG'*inv(S_MAG);
        P_MAG = (eye(9) - K_MAG*H_MAG)*P_MAG;
        
        mx = (mag_mx -M_BIAS(1));
        my = (mag_my -M_BIAS(2));
        mz = (mag_mz -M_BIAS(3));
        n_m = norm([mx my mz]);
        % 
        mx = mx/n_m +M_BIAS(1);
        my = my/n_m +M_BIAS(2);
        mz = mz/n_m +M_BIAS(3);

        new_mag = [mx my mz mag_mx mag_my mag_mz]';
        %new_mag = [mag_mx mag_my mag_mz mag_mx mag_my mag_mz]';
        
        z_m = (new_mag);
        z_m2 = H_MAG*X_MAG;
        
        Z_MAG = z_m - z_m2;
        X_MAG = X_MAG + K_MAG*Z_MAG;
        
        M_BIAS  = X_MAG(4:6)';
        M_SCALE = X_MAG(7:9)';
        
        mag_mx = (mag_mx -M_BIAS(1))*M_SCALE(1);
        mag_my = (mag_my -M_BIAS(2))*M_SCALE(2);
        mag_mz = (mag_mz -M_BIAS(3))*M_SCALE(3);
        MAGCSV(magCount,:) = [ii X_MAG' Z_MAG' sqrt(diag(P_MAG))'  sqrt(diag(S_MAG))'];
        
        REF_MAGCSV(magCount,:) = [ii MAG_BIAS' MAG_SCALE' new_mag'];
        magCount = magCount +1;
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
        %[val, idx] = max(p881L_data.Intensities(p881L_idx,:)); 
        
        p881l_intensities = zeros(1,500);
        ramp_ratio = 1:-1/500:0;
        for jj=1:1:500
            p881l_intensities(1,jj) = (ramp_ratio(jj)*p881L_data.Intensities(p881L_idx,jj));
        end
        [val, idx] = max(p881l_intensities); 

        range = double(p881L_data.Range(p881L_idx,1)/1000);
        p881L.intensities  = double(range*idx/500);
        p881L.currentAngle = p881L_data.Angle(p881L_idx,1) + pi/2;

        body_pointX = 0;
        body_pointY =  p881L.intensities*cos(p881L.currentAngle);
        body_pointZ =  p881L.intensities*sin(p881L.currentAngle);
     end
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Treat the new sensor data
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if imu_newData
        imu_newData = true;

        Timu = (ii - dt_imu);
        dt_imu = ii;
        
        g = norm([body_ax body_ay body_az]);
        ax = body_ax/g;
        ay = body_ay;
        az = body_az;

        accel_angle(1) = atan(ay/az);
        accel_angle(2) = asin(ax);

        body_ax =  body_ax -BIAS(1);
        body_ay =  body_ay -BIAS(2);
        body_az =  body_az -BIAS(3);
        
        body_gx =  body_gx -BIAS(4);
        body_gy =  body_gy -BIAS(5);
        body_gz =  body_gz -BIAS(6);
 
        %alpha = 0.99;
        %angle_filtered(1:2) = alpha*(angle_filtered(1:2)) + (1-alpha)*(accel_angle(1:2));

        %rad2deg(angle_filtered(1:2))
    end
    
    if mag_newData 
        %mag_mx =  ( mag_mx - MAG_BIAS(1))/MAG_SCALE(1);
        %mag_my =  ( mag_my - MAG_BIAS(2))/MAG_SCALE(2);
        %mag_mz =  ( mag_mz - MAG_BIAS(3))/MAG_SCALE(3);

        if first_run
            X_INS(9) = MAG_DEC + getHeading(mag_mx,mag_my);
            first_run = false;
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % KF-BASED
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if p881L_newData
        T_KF = (ii - dt_KF);
        dt_KF = ii;

        U_IMU = [body_ax;body_ay;body_az;body_gx;body_gy;body_gz];
        U_INS = U_IMU;
       
        %[A,B] = INSMatrices(X_INS(9),X_INS(8),X_INS(7),T_KF);
        %X_INS = A*X_INS + B*U_INS;% + [ 0 0 9.81*T_KF 0 0 (9.81*T_KF*T_KF)/2 0 0 0]';
        
        X_INS = getState(X_INS,U_INS,T_KF);%ODE45

        [Ad,Bd] = dynamicsMatrices(X_INS(9),X_INS(8),X_INS(7),T_KF,1);

        Pk = Ad*Pk*Ad'+  T_KF*Bd*Q*Bd' + betta*P0;
        
        Pk_1 = Pk;
        
    end
       
    if  imu_newData
        imu_newData = false;

        C1 = [0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
              0 0 0 0 0 0 0 1 0 0 0 0 0 0 0];
    
        R1 = diag([R(7,7) R(8,8)]);
        
        S = C1*Pk*(C1') + R1;
        Gk = Pk*(C1')*inv(S);
        Pk = (eye(15) - Gk*C1)*Pk;
        
       
        new_roll  = angle_filtered(1);
        new_pitch = angle_filtered(2);  

        Y  = [new_roll new_pitch]';
        dY = angdiff(Y,[X_INS(7) X_INS(8)]');
        %dY = [X_INS(7) X_INS(8)]' - Y;
                
        X_E = [0;0;0;0;0;0;0;0;0;BIAS] + Gk*dY;

        aux =  angdiff(X_E(7:8),X_INS(7:8));
        X_INS = X_INS - X_E(1:9);
        X_INS(7:8) = aux;

        X_INS = X_INS - X_E(1:9);
        BIAS =  [X_E(10:end)];
        
        
        X_EP(7:8) = X_E(7:8);
        IK(7:8)   = dY;
        STD_IK(7:8) = sqrt(diag(S));
    end
    
    if  mag_newData
        mag_newData = false;

        C1 = [0 0 0 0 0 0 0 0 1 0 0 0 0 0 0];
    
        R1 = diag([R(9,9)]);
        
        S = C1*Pk*(C1') + R1;
        Gk = Pk*(C1')*inv(S);
        Pk = (eye(15) - Gk*C1)*Pk;
        
        roll  = angle_filtered(1);
        pitch = angle_filtered(2);
        
        mm = getRotation(0,pitch,roll)*[mag_mx mag_my mag_mz]';
        new_yaw   = MAG_DEC + getHeading(mm(1),mm(2));

        Y  = [new_yaw]';
        dY = angdiff(Y,X_INS(9));
        X_E = [0;0;0;0;0;0;0;0;0;BIAS] + Gk*dY;
        
        aux =  angdiff(X_E(9),X_INS(9));
        X_INS = X_INS - X_E(1:9);
        X_INS(9) = aux;
        BIAS =  [X_E(10:end)];
        
        X_EP(9) = X_E(9);
        IK(9)   = dY;
        STD_IK(9) = sqrt(diag(S));
    end

    if  depth_newData
        depth_newData = false;

        C1 = [0 0 0 0 0 1 0 0 0 0 0 0 0 0 0];
    
        R1 = diag([R(6,6)]);
        
        S = C1*Pk*(C1') + R1;
        Gk = Pk*(C1')*inv(S);
        Pk = (eye(15) - Gk*C1)*Pk;
        
        new_depth = depth_pz;

        Y  = [new_depth]';
        dY = [X_INS(6)]' - Y;
                
        X_E = [0;0;0;0;0;0;0;0;0;BIAS] + Gk*dY;
        X_INS = X_INS - X_E(1:9);
        BIAS =  [X_E(10:end)];
        
        X_EP(6) = X_E(6);
        IK(6)   = dY;
        STD_IK(6) = sqrt(diag(S));
    end

    if  dvl_newData
        
        dvl_newData = false;

        C1 = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
              0 1 0 0 0 0 0 0 0 0 0 0 0 0 0
              0 0 1 0 0 0 0 0 0 0 0 0 0 0 0];
    
        R1 = diag([R(1,1) R(2,2) R(3,3)]);

        Rypr = getRotation(X_INS(9),X_INS(8),X_INS(7));
        
        S = C1*Pk*(C1') + Rypr*R1*Rypr';
        Gk = Pk*(C1')*inv(S);
        Pk = (eye(15) - Gk*C1)*Pk;

        %new sensor values
        new_vel   = Rypr*[body_vx body_vy body_vz]';
            
        Y  = [new_vel']';
        dY = [X_INS(1) X_INS(2) X_INS(3)]' - Y;
            
        X_E = [0;0;0;0;0;0;0;0;0;BIAS] + Gk*dY;
        X_INS = X_INS - X_E(1:9);
        BIAS =  [X_E(10:end)];
        
        X_EP(1:3) = X_E(1:3);
        IK(1:3)   = dY;
        STD_IK(1:3) = sqrt(diag(S));
    end
 

   if  gps_newData
        gps_newData = false;

        C1 = [0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
              0 0 0 0 1 0 0 0 0 0 0 0 0 0 0];
    
        R1 = diag([R(4,4) R(5,5)]);
        
        S = C1*Pk*(C1') + R1;
        Gk = Pk*(C1')*inv(S);
        Pk = (eye(15) - Gk*C1)*Pk;

        %new sensor values
        new_pos   = [gps_px gps_py]';
            
        Y  = [new_pos']';
        dY = [X_INS(4) X_INS(5)]' - Y;
            
        X_E = [0;0;0;0;0;0;0;0;0;BIAS] + Gk*dY;
        X_INS = X_INS - X_E(1:9);
        BIAS =  [X_E(10:end)];
        
        X_EP(4:5) = X_E(4:5);
        IK(4:5)   = dY;
        STD_IK(4:5) = sqrt(diag(S));
   end

    %GENERATE POINT CLOUD
    if  p881L_newData
        p881L_newData = false;
        
        global_point_KF = X_INS(4:6) + getRotation(X_INS(9),X_INS(8),X_INS(7))*[body_pointX body_pointY body_pointZ]';
        KFCSV(p881L_idx,:) = [(ii),X_INS' global_point_KF' p881L.intensities p881L.currentAngle];

        KFCSV_XE(p881L_idx,:) = [(ii)  X_EP' BIAS'];
        KFCSV_XED(p881L_idx,:) = [(ii) sqrt(diag(Pk))'];
        mat_Pk_1(p881L_idx,:) = reshape(Pk_1,1,225);
        mat_Pk(p881L_idx,:)   = reshape(Pk,1,225);
        mat_Ik(p881L_idx,:)   = [(ii)  IK'];
        mat_STD_Ik(p881L_idx,:)   = [(ii)  STD_IK'];
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
save(path+pasta+'KFCSV.mat','KFCSV');
save(path+pasta+'KFCSV_XE.mat','KFCSV_XE');
save(path+pasta+'KFCSV_XED.mat','KFCSV_XED');
save(path+pasta+'mat_Pk.mat','mat_Pk');
save(path+pasta+'mat_Pk_1.mat','mat_Pk_1');
save(path+pasta+'mat_Ik.mat','mat_Ik')
save(path+pasta+'mat_STD_Ik.mat','mat_STD_Ik')

save('MAGCSV.mat','MAGCSV');
save('REF_MAGCSV.mat','REF_MAGCSV');

%end