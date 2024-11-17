clc;clear all;close all;
addpath('./functions')
load("MAIN.mat")
path = "";
pasta = MAIN.folder;
is_sim = 0;

switch MAIN.DATASET
    case DATASET.SIM_CAPAO_PRETO
        is_sim = 1;
    case DATASET.SIM_JAGUARI
        is_sim = 1;
    case DATASET.SIM_MONTE_CLARO
        is_sim = 1;
    otherwise
        disp("Not a simulation path")
        return
end

dataset = MAIN.DATASET;

isSaveFigure = 0;
isSaveData   = 1;

noise_gps     = [0.01 0.01];
noise_depth   =  0.1;
noise_dvl     = [0.01 0.01 0.01];
noise_acc     = [0.1 0.1 0.1];
noise_giro    = [0.01 0.01 0.01];
noise_rpy     = [deg2rad(5) deg2rad(5) deg2rad(5)];
bias_acc      = [0.02 0.02 0.02];
bias_gyro     = [0.003 0.003 0.003];

noise_mag    = [10 10 10];
MAG_DEC   = deg2rad(-10);
MAG_SCALE = [3000 5000 2000];
MAG_BIAS  = [ 100 200 100];

fs = 10;
g = 9.81;

%% DO NOT EDIT HERE
rng("shuffle")

switch dataset
    case DATASET.SIM_CAPAO_PRETO
        [pos,vel,q,acel,giro,t] = capaoPreto();
    case DATASET.SIM_JAGUARI
        disp('aqui')
        [pos,vel,q,acel,giro,t] = jaguari();
    case DATASET.SIM_MONTE_CLARO
        [pos,vel,q,acel,giro,t] = monteClaro();
    otherwise
        disp('Simulation not implemented')
        return
end


body_acc      = quatrotate(q,acel);
body_acc(:,3) = body_acc(:,3) + g*ones(length(body_acc),1);
body_giro     = quatrotate(q,giro);

%eul           = quat2eul(q);


% velocity = zeros(N,3);
% position = zeros(N,3);
% v = [0 0 0]';
% p = [0 0 0]';
% dt = 1/fs;
% N = length(body_acc);
% for ii=1:1:N
%     R   = getRotation(eul(ii,1),eul(ii,2),eul(ii,3));
%     acc = R*body_acc(ii,:)' + [0;0;g];
%     v   = v +dt*acc;
%     p   = p +v*dt + acc*dt^2/2.0; 
%     velocity(ii,:) = v';
%     position(ii,:) = p';
% end

N = length(body_acc);

GPS_data(:,1) =  pos(:,1) + noise_gps(1)*randn(N,1);
GPS_data(:,2) =  pos(:,2) + noise_gps(2)*randn(N,1);
GPS_data(:,3) =  pos(:,3) + noise_depth*randn(N,1);

body_vel = quatrotate(q,vel);
body_vel(:,1) = body_vel(:,1) + noise_dvl(1)*randn(N,1);
body_vel(:,2) = body_vel(:,2) + noise_dvl(2)*randn(N,1);
body_vel(:,3) = body_vel(:,3) + noise_dvl(3)*randn(N,1);
body_vel      = [body_vel ones(N,1)];

body_acc(:,1) = body_acc(:,1) + noise_acc(1)*randn(N,1) + bias_acc(1);
body_acc(:,2) = body_acc(:,2) + noise_acc(2)*randn(N,1) + bias_acc(2);
body_acc(:,3) = body_acc(:,3) + noise_acc(3)*randn(N,1) + bias_acc(3);

body_giro(:,1) = body_giro(:,1) + noise_giro(1)*randn(N,1) + bias_gyro(1);
body_giro(:,2) = body_giro(:,2) + noise_giro(2)*randn(N,1) + bias_gyro(2);
body_giro(:,3) = body_giro(:,3) + noise_giro(3)*randn(N,1) + bias_gyro(3);



rpy = quat2eul(q);

rpy(:,1) = rpy(:,1) - MAG_DEC;
rpy(:,2) = rpy(:,2);
rpy(:,3) = rpy(:,3);


mag      = zeros(N,3);

theta = zeros(N,1);
phi   = zeros(N,1);
yaw   = zeros(N,1);

for ii=1:1:N
    Rm = getRotation(0,rpy(ii,2),rpy(ii,3))';
    
    my = -sin(rpy(ii,1));
    mx = cos(rpy(ii,1));

    bm = Rm*[mx my 0]';
    bm = diag(MAG_SCALE)*bm;
    bm = bm + MAG_BIAS';

    mag(ii,1) = bm(1) + noise_mag(1)*randn();
    mag(ii,2) = bm(2) + noise_mag(2)*randn();
    mag(ii,3) = bm(3) + noise_mag(3)*randn();
 
    body_ax = body_acc(ii,1);
    body_ay = body_acc(ii,2);
    body_az = body_acc(ii,3);
    g = norm([body_ax body_ay body_az]);
    
    ax = body_ax/g;
    ay = body_ay/g;
    az = body_az/g;

    phi(ii)   = atan(ay/az);
    theta(ii) = asin(ax);

    Rm = getRotation(0,phi(ii),theta(ii))';
    M = inv(diag(MAG_SCALE))*(mag(ii,:)' - MAG_BIAS');
    M = Rm*M;
    yaw(ii)   = getHeading(M(1),M(2));
end

rad2deg(std(theta))
rad2deg(std(phi))

rpy(:,1) = rpy(:,1) + noise_rpy(1)*randn(N,1) + MAG_DEC;
rpy(:,2) = rpy(:,2) + noise_rpy(2)*randn(N,1);
rpy(:,3) = rpy(:,3) + noise_rpy(3)*randn(N,1);

% figure
% eul = quat2eul(q);
% 
% subplot(3,1,1)
% hold on;
% plot(t,rad2deg(theta));
% plot(t,rad2deg(eul(:,3)));
% grid on
% 
% subplot(3,1,2)
% hold on
% plot(t,rad2deg(phi));
% plot(t,rad2deg(eul(:,2)));
% grid on
% 
% subplot(3,1,3)
% hold on
% plot(t,rad2deg(yaw));
% plot(t,rad2deg(eul(:,1)));
% grid on


range_max = 10;
np881l = length(vel);

Intensities = zeros(np881l,500);
Intensities(:,150) = 10;
Angle = zeros(np881l,1);

angle = 0;
dTheta = (pi/180);
inc = -dTheta;

%t2 = 0:0.1/3:t(end);

switch dataset
    case DATASET.SIM_CAPAO_PRETO
        %Capão Preto
        do = 0;
        do1 = -pi/2;
        for ii=1:1:np881l 
            angle = angle + inc;
        
            if angle > 0
               angle = 0;
               inc = -dTheta;
            else
                if angle < -pi/2
                    angle = -pi/2;
                    inc = dTheta;
                 end
            end
            
            if t(ii)> 1230
                do = -pi/2;
            end
            Angle(ii) = angle + do + do1;
            Intensities(ii,150) = 10;
        end
    otherwise
        %Jaguari/Monte Claro
        for ii=1:1:np881l
            angle = angle + inc;
            Angle(ii) = angle;
            %Intensities(ii,100) = 10;
        end
end

Angle(:,1) = Angle(:,1) + 3*pi/2;
Range = (range_max*1000)*ones(np881l,1);

 % DRCSV = zeros(length(pos),3);
 % rpy = quat2eul(q);
 %     for ii = 1:1:length(pos)/2
 %         x     = pos(ii,1);
 %         y     = pos(ii,2);
 %         z     = pos(ii,3);
 % 
 %         px  = 0;
 %         py  = 3*cos(Angle(ii));
 %         pz  = 3*sin(Angle(ii));
 % 
 %         normal = [px py pz]'+ [x y z]';
 % 
 %         DRCSV(ii,:) = [normal'];
 %     end

% figure
% pcshow([DRCSV(:,1),DRCSV(:,2),DRCSV(:,3)],"MarkerSize",5,BackgroundColor=[1 1 1]);
% 
% hold on;
% plot3([0 10], [0 0], [0 0],"Color","red","LineWidth",1)
% plot3([0 0], [0 3], [0 0],"Color","green","LineWidth",1)
% plot3([0 0], [0 0], [0 3],"Color","blue","LineWidth",1)
% plot3([0 150], [0 0], [0 0],"k--")
% 
% xlabel('X')
% ylabel('Y')
% zlabel('Z')



% csvwrite('capaoPreto.csv', DRCSV);
%saveas(h,path+pasta+"\figs\"+'simTraj_ptCloud.png')

%% Plot DATA
h = figure('Name','Position');

fig1 = subplot(3,1,1);
plot(t,pos(:,1));grid on;xlabel('Tempo (s)');ylabel('P_x (m)');
title("Posição no eixo X")

fig2 = subplot(3,1,2);
plot(t,pos(:,2)*0);grid on;xlabel('Tempo (s)');ylabel('P_y (m)');
title("Posição no eixo Y")

fig3 = subplot(3,1,3);
plot(t,pos(:,3));grid on;xlabel('Tempo (s)');ylabel('P_z (m)');
title("Posição no eixo Z")
set(h, 'WindowStyle', 'Docked');

saveas(h,path+pasta+"\figs\"+'simTraj_pos.png')
if isSaveFigure
    printFig(fig1,path+pasta+"\figs\"+'PositionX');
    printFig(fig2,path+pasta+"\figs\"+'PositionY');
    printFig(fig3,path+pasta+"\figs\"+'PositionZ');
end
%%
h = figure('Name','Velocity');

fig1 = subplot(3,1,1);
plot(t,vel(:,1));grid on;xlabel('Tempo (s)');ylabel('v_x (m/s)');
title("Velocidade no eixo X")

fig2 = subplot(3,1,2);
plot(t,vel(:,2));grid on;xlabel('Tempo (s)');ylabel('V_y (m/s)');
title("Velocidade no eixo Y")

fig3 = subplot(3,1,3);
plot(t,vel(:,3));grid on;xlabel('Tempo (s)');ylabel('V_z (m/s)');
title("Velocidade no eixo Z")
set(h, 'WindowStyle', 'Docked');

saveas(h,path+pasta+"\figs\"+'simTraj_vel.png')
if isSaveFigure
    printFig(fig1,path+pasta+"\figs\"+'VelocityX');
    printFig(fig2,path+pasta+"\figs\"+'VelocityY');
    printFig(fig3,path+pasta+"\figs\"+'VelocityZ');
end
%% 

h = figure('Name','Euler Angles');
eul = rad2deg(quat2eul(q));


fig1 = subplot(3,1,1);
plot(t,eul(:,3));grid on;xlabel('Tempo (s)');ylabel('\phi (°)');
title("Ângulo de rolamento (Eixo X)")

fig2 =  subplot(3,1,2);
plot(t,eul(:,2));grid on;xlabel('Tempo (s)');ylabel('\theta (°)');
title("Ângulo de arfagem  (Eixo Y)")

fig3 = subplot(3,1,3);
plot(t,eul(:,1));grid on;xlabel('Tempo (s)');ylabel('\psi (°)');
title("Ângulo de guinada  (Eixo Z)")
set(h, 'WindowStyle', 'Docked');

saveas(h,path+pasta+"\figs\"+'simTraj_eul.png')
if isSaveFigure
    printFig(fig1,path+pasta+"\figs\"+'eulerAnglesRoll');
    printFig(fig2,path+pasta+"\figs\"+'eulerAnglesPtitch');
    printFig(fig3,path+pasta+"\figs\"+'eulerAnglesYaw');
end
%%
h = figure('Name','Aceleration');

fig1 = subplot(3,1,1);
plot(t,acel(:,1));grid on;xlabel('Tempo (s)');ylabel('a_x (m/s^2)');
title("Aceleração (Eixo X)")
fig2 = subplot(3,1,2);
plot(t,acel(:,2));grid on;xlabel('Tempo (s)');ylabel('a_y (m/s^2)');
title("Aceleração (Eixo Y)")
fig3 = subplot(3,1,3);
plot(t,acel(:,3));grid on;xlabel('Tempo (s)');ylabel('a_z (m/s^2)');
title("Aceleração (Eixo Z)")
set(h, 'WindowStyle', 'Docked');

saveas(h,path+pasta+"\figs\"+'simTraj_accel_raw.png')
if isSaveFigure
    printFig(fig1,path+pasta+"\figs\"+'AcelerationX');
    printFig(fig2,path+pasta+"\figs\"+'AcelerationY');
    printFig(fig3,path+pasta+"\figs\"+'AcelerationZ');
end
%% 

h = figure('Name','Angular Velocity');

fig1 = subplot(3,1,1);
plot(t,giro(:,1));grid on;xlabel('Tempo (s)');ylabel('\omega _x (rad/s)');
title("Velocidade angular (Eixo X)")
fig2 = subplot(3,1,2);
plot(t,giro(:,2));grid on;xlabel('Tempo (s)');ylabel('\omega _y (rad/s)');
title("Velocidade angular (Eixo Y)")
fig3 = subplot(3,1,3);
plot(t,giro(:,3));grid on;xlabel('Tempo (s)');ylabel('\omega _z (rad/s)');
title("Velocidade angular (Eixo Z)")
set(h, 'WindowStyle', 'Docked');

saveas(h,path+pasta+"\figs\"+'simTraj_angVel_raw.png')
if isSaveFigure
    printFig(fig1,path+pasta+"\figs\"+'angularVelocityX');
    printFig(fig2,path+pasta+"\figs\"+'angularVelocityY');
    printFig(fig3,path+pasta+"\figs\"+'angularVelocityZ');
end
%% Plot Body DATA
h = figure('Name','Body Aceleration');

fig1 = subplot(3,1,1);
plot(t,body_acc(:,1));grid on;xlabel('Tempo (s)');ylabel('a_{bx} (m/s^2)')
title("Medida gerada de aceleração no corpo (Eixo X)")
fig2 = subplot(3,1,2);
plot(t,body_acc(:,2));grid on;xlabel('Tempo (s)');ylabel('a_{by} (m/s^2)')
title("Medida gerada de aceleração no corpo (Eixo Y)")
fig3 = subplot(3,1,3);
plot(t,body_acc(:,3));grid on;xlabel('Tempo (s)');ylabel('a_{bz} (m/s^2)')
title("Medida gerada de aceleração no corpo  (Eixo Z)")
set(h, 'WindowStyle', 'Docked');

saveas(h,path+pasta+"\figs\"+'simTraj_bodyAccel.png')
if isSaveFigure
    printFig(fig1,path+pasta+"\figs\"+'bodyAcelerationX');
    printFig(fig2,path+pasta+"\figs\"+'bodyAcelerationY');
    printFig(fig3,path+pasta+"\figs\"+'bodyAcelerationZ');
end
%% 
h = figure('Name','Body Giro');

fig1 = subplot(3,1,1);
plot(t,body_giro(:,1));grid on;xlabel('Tempo (s)');ylabel('\omega _{bx} (rad/s)');
title("Medida gerada de velocidade angular no corpo  (Eixo X)")
fig2 = subplot(3,1,2);
plot(t,body_giro(:,2));grid on;xlabel('Tempo (s)');ylabel('\omega _{by} (rad/s)');
title("Medida gerada de velocidade angular no corpo  (Eixo Y)")
fig3 = subplot(3,1,3);
plot(t,body_giro(:,3));grid on;xlabel('Tempo (s)');ylabel('\omega _{bz} (rad/s)');
title("Medida gerada de velocidade angular no corpo  (Eixo Z)")
set(h, 'WindowStyle', 'Docked');

saveas(h,path+pasta+"\figs\"+'simTraj_bodyGyro.png')
if isSaveFigure
    printFig(fig1,path+pasta+"\figs\"+'bodyGiroX');
    printFig(fig2,path+pasta+"\figs\"+'bodyGiroY');
    printFig(fig3,path+pasta+"\figs\"+'bodyGiroZ');
end
%% 
h = figure('Name','Body Velocity');

fig1 = subplot(3,1,1);
plot(t,body_vel(:,1));grid on;xlabel('Tempo (s)');ylabel('v _{bx} (m/s)');
title("Medida gerada de velocidade no corpo  (Eixo X)")
fig2 = subplot(3,1,2);
plot(t,body_vel(:,2));grid on;xlabel('Tempo (s)');ylabel('v _{by} (m/s)');
title("Medida gerada de velocidade no corpo  (Eixo Y)")
fig3 =subplot(3,1,3);
plot(t,body_vel(:,3));grid on;xlabel('Tempo (s)');ylabel('v _{bz} (m/s)');
title("Medida gerada de velocidade no corpo  (Eixo Z)")
set(h, 'WindowStyle', 'Docked');

saveas(h,path+pasta+"\figs\"+'simTraj_bodyVel.png')
if isSaveFigure
    printFig(fig1,path+pasta+"\figs\"+'bodyVelocityX');
    printFig(fig2,path+pasta+"\figs\"+'bodyVelocityY');
    printFig(fig3,path+pasta+"\figs\"+'bodyVelocityZ');
end
% %% Plot Recovery DATA
% 
% h = figure('Name','Recovery Velocity');
% fig1 = subplot(3,1,1);
% plot(t,velocity(:,1));grid on;xlabel('Time (s)');hold on;
% plot(t,vel(:,1));grid on;xlabel('Time (s)');
% ylabel('v_{x} (m/s)')
% legend('Recovery','Ground Truth')
% 
% fig2 = subplot(3,1,2);
% plot(t,velocity(:,2));grid on;xlabel('Time (s)');hold on;
% plot(t,vel(:,2));grid on;xlabel('Time (s)');
% ylabel('v_{y} (m/s)');
% legend('Recovery','Ground Truth')
% 
% fig3 = subplot(3,1,3);
% plot(t,velocity(:,3));grid on;xlabel('Time (s)');hold on;
% plot(t,vel(:,3));grid on;xlabel('Time (s)');
% ylabel('v_{z} (m/s)');
% legend('Recovery','Ground Truth')
% set(h, 'WindowStyle', 'Docked');
% 
% if isSaveFigure
%     printFig(fig1,path+pasta+"\figs\"+'recoveryVelocityX');
%     printFig(fig2,path+pasta+"\figs\"+'recoveryVelocityY');
%     printFig(fig3,path+pasta+"\figs\"+'recoveryVelocityZ');
% end
% %%
% h = figure('Name','Recovery Position');
% 
% fig1 = subplot(3,1,1);
% plot(t,position(:,1));grid on;xlabel('Time (s)');hold on;
% plot(t,pos(:,1));grid on;xlabel('Time (s)');
% ylabel('p_{x} (m)')
% legend('Recovery','Ground Truth')
% 
% fig2 = subplot(3,1,2);
% plot(t,position(:,2));grid on;xlabel('Time (s)');hold on;
% plot(t,pos(:,2));grid on;xlabel('Time (s)');
% ylabel('p_{y} (m)');
% legend('Recovery','Ground Truth')
% 
% fig3 = subplot(3,1,3);
% plot(t,position(:,3));grid on;xlabel('Time (s)');hold on;
% plot(t,pos(:,3));grid on;xlabel('Time (s)');
% ylabel('p_{z} (m)');
% legend('Recovery','Ground Truth')
% set(h, 'WindowStyle', 'Docked');
% 
% if isSaveFigure
%     printFig(fig1,path+pasta+"\figs\"+'recoveryPositonX');
%     printFig(fig2,path+pasta+"\figs\"+'recoveryPositonY');
%     printFig(fig3,path+pasta+"\figs\"+'recoveryPositonZ');
% end
%%
h2 = figure('Name','GPS data');

fig1 = subplot(3,1,1);
plot(t,GPS_data(:,1));grid on;xlabel('Tempo (s)');hold on;
ylabel('gps_{x} (m)')
title("Medida gerada do GPS (Eixo X)")
fig2 = subplot(3,1,2);
plot(t,GPS_data(:,2));grid on;xlabel('Tempo (s)');hold on;
%plot(t,pos(:,2));grid on;xlabel('Tempo (s)');
ylabel('gps_{y} (m)');
title("Medida gerada do GPS (Eixo Y)")

fig3 = subplot(3,1,3);
plot(t,GPS_data(:,3));grid on;xlabel('Tempo (s)');hold on;
ylabel('p_{z} (m)');
title("Medida gerada do sensor de Pressão (Eixo Z)")
set(h2, 'WindowStyle', 'Docked');

saveas(h,path+pasta+"\figs\"+'simTraj_GPS_DEPHT.png')
if isSaveFigure
    printFig(fig1,path+pasta+"\figs\"+'gpsX');
    printFig(fig2,path+pasta+"\figs\"+'gpsY');
    printFig(fig3,path+pasta+"\figs\"+'gpsZ');
end
%% 
h2 = figure('Name','Mag data');

fig1 = subplot(2,2,1);
plot(t,mag(:,1));grid on;xlabel('Tempo (s)');
ylabel('m_{x} (T)')
title("Medida gerada para o magnetômetro (Eixo X)")
fig2 = subplot(2,2,2);
plot(t,mag(:,2));grid on;xlabel('Tempo (s)');
ylabel('m_{y} (T)');
title("Medida gerada para o magnetômetro (Eixo Y)")
fig3 = subplot(2,2,3);
plot(t,mag(:,3));grid on;xlabel('Tempo (s)');
ylabel('m_{z} (T)');
title("Medida gerada para o magnetômetro (Eixo Z)")
fig4 = subplot(2,2,4);
plot3(mag(:,1),mag(:,2),mag(:,3),"*");grid on;
title("Medida gerada para o magnetômetro (Eixos X,Y e Z)")
xlabel('m_{x} (T)');
ylabel('m_{y} (T)');
zlabel('m_{z} (T)');
set(h2, 'WindowStyle', 'Docked');

saveas(h,path+pasta+"\figs\"+'simTraj_mag.png')
if isSaveFigure
    printFig(fig1,path+pasta+"\figs\"+'magX');
    printFig(fig2,path+pasta+"\figs\"+'magY');
    printFig(fig3,path+pasta+"\figs\"+'magZ');
    printFig(fig4,path+pasta+"\figs\"+'magXYZ');
end

%% Save Data
if isSaveData==1
    p881L_data = timetable(Intensities,Angle,Range,'SampleRate',fs);
    save(path+pasta+'p881L_data.mat','p881L_data');
    
    IMU_TABLE = [body_acc body_giro];
    imu_data      = timetable(IMU_TABLE,'SampleRate',fs);
    save(path+pasta+'imu_data.mat','imu_data');
    
    DVL_TABLE = body_vel;
    dvl_data      = timetable(DVL_TABLE,'SampleRate',fs);
    save(path+pasta+'dvl_data.mat','dvl_data');
    
    MAG_TABLE = mag;
    mag_data      = timetable(MAG_TABLE,'SampleRate',fs);
    save(path+pasta+'mag_data.mat','mag_data');
    
    PRESSURE_TABLE = GPS_data(:,3);
    depth_data      = timetable(PRESSURE_TABLE,'SampleRate',fs);
    save(path+pasta+'depth_data.mat','depth_data');
    
    MAG_CALIBRATION.BIAS   = MAG_BIAS;
    MAG_CALIBRATION.SCALE  = MAG_SCALE;
    MAG_CALIBRATION.DEC    = MAG_DEC;
    
    save(path+pasta+'mag_cal.mat','MAG_CALIBRATION');
    
    RPY_TABLE     = rpy;
    rpy_data      = timetable(RPY_TABLE,'SampleRate',fs);
    save(path+pasta+'rpy_data.mat','rpy_data');
    
    gps = GPS_data;
    gps_data      = timetable(gps,'SampleRate',fs);
    save(path+pasta+'gps_data.mat','gps_data');
    
    %% Ground Truth
    data = pos;
    gt_pos      = timetable(data,'SampleRate',fs);
    save(path+pasta+'gt_pos.mat','gt_pos');

    data = vel;
    gt_vel      = timetable(data,'SampleRate',fs);
    save(path+pasta+'gt_vel.mat','gt_vel');

    data = quat2eul(q);
    gt_eul      = timetable(data,'SampleRate',fs);
    save(path+pasta+'gt_eul.mat','gt_eul');

    data = [bias_acc(1)*ones(N,1) bias_acc(2)*ones(N,1) (bias_acc(3)*ones(N,1))];
    gt_bias_accel  = timetable(data,'SampleRate',fs);
    save(path+pasta+'gt_bias_accel.mat','gt_bias_accel');

    data = [bias_gyro(1)*ones(N,1) bias_gyro(2)*ones(N,1) bias_gyro(3)*ones(N,1)];
    gt_bias_gyro  = timetable(data,'SampleRate',fs);
    save(path+pasta+'gt_bias_gyro.mat','gt_bias_gyro');

end
