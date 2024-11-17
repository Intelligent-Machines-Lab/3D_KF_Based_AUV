function plotData()
clc;close all;clear all;
addpath('./functions')

load("MAIN.mat")
path = "";
pasta = MAIN.folder;

load(path+pasta+"DRCSV.mat")
load(path+pasta+"KFCSV.mat")
load(path+pasta+"KFCSV_XE.mat")
load(path+pasta+"KFCSV_XED.mat")
load(path+pasta+"gps_data.mat")
load(path+pasta+"mat_Pk.mat")
load(path+pasta+"mat_Pk_1.mat")
load(path+pasta+"mat_Ik.mat")
load(path+pasta+"mat_STD_Ik.mat")

if MAIN.is_sim == 1
    load(path+pasta+"gt_pos.mat")
    load(path+pasta+"gt_vel.mat")
    load(path+pasta+"gt_bias_gyro.mat")
    load(path+pasta+"gt_bias_accel.mat")
    load(path+pasta+"gt_eul.mat")
else
    gt_pos = [];
    gt_vel = [];
    gt_bias_gyro = [];
    gt_bias_accel = [];
    gt_eul = [];
    
    if MAIN.isGPS
        gt_pos.Time = gps_data.Time;
        gt_pos.data = gps_data.gps;
        gt_pos.data(:,3) = 0*gt_pos.data(:,3);
    end
end

cuttoff = 10;
KFCSV     = KFCSV(1:1:end-cuttoff,:);
KFCSV_XE  = KFCSV_XE(1:1:end-cuttoff,:);
KFCSV_XED = KFCSV_XED(1:1:end-cuttoff,:); 
mat_Ik =mat_Ik(1:1:end-cuttoff,:);
mat_STD_Ik = mat_STD_Ik(1:1:end-cuttoff,:);


plotVelocity(KFCSV,KFCSV_XED,gt_vel)
plotPosition(KFCSV,KFCSV_XED,gt_pos)
plotEulerAngles(KFCSV,KFCSV_XED,gt_eul)
plotBiasAccel(KFCSV_XE,KFCSV_XED,gt_bias_accel)
plotBiasGyro(KFCSV_XE,KFCSV_XED,gt_bias_gyro)

plotInovationVelocity(mat_Ik,mat_STD_Ik)
plotInovationPosition(mat_Ik,mat_STD_Ik)
plotInovationEulerAngles(mat_Ik,mat_STD_Ik)

plot_KF_DR_GT_Position(KFCSV,DRCSV,gt_pos)
plot_KF_DR_GT_Orientation(KFCSV,DRCSV,gt_pos)
plotVelocityDR(DRCSV,gt_vel)
plotPositionDR(DRCSV,gt_pos)
plotEulerAnglesDR(DRCSV,gt_eul)


figure
pcshow([KFCSV(:,11),KFCSV(:,12),KFCSV(:,13)],"MarkerSize",15,BackgroundColor=[1 1 1]);hold on;
colorbar(Color=[0 0 0])
plot3([0 10], [0 0], [0 0],"Color","red","LineWidth",1)
plot3([0 0], [0 3], [0 0],"Color","green","LineWidth",1)
plot3([0 0], [0 0], [0 3],"Color","blue","LineWidth",1)

plot3(KFCSV(:,5),KFCSV(:,6),KFCSV(:,7),"k--")

xlabel("X (m)")
ylabel("Y (m)");
zlabel("Z (m)")
title("Nuvem de pontos obtida utilizando a trajetória gerada pelo Filtro de Kalman Estendido")
grid on

if MAIN.is_sim
    errorDR = errorTraj([seconds(gt_pos.Time) gt_pos.data],[DRCSV(:,1) DRCSV(:,2:4)]);
    errorKF = errorTraj([seconds(gt_pos.Time) gt_pos.data],[KFCSV(:,1) KFCSV(:,5:7)]);
    plotErrorPosition(errorKF,errorDR);
    

    errorAngleDR = errorAngle([seconds(gt_eul.Time) gt_eul.data],[DRCSV(:,1) DRCSV(:,8:10)]);
    errorAngleKF = errorAngle([seconds(gt_eul.Time) gt_eul.data],[KFCSV(:,1) KFCSV(:,8:10)]);
    plotErrorAngle(errorAngleKF,errorAngleDR);

end

%%
figure

pcshow([DRCSV(:,11),DRCSV(:,12),DRCSV(:,13)],"MarkerSize",15,BackgroundColor=[1 1 1]);hold on;
colorbar(Color=[0 0 0])
plot3([0 10], [0 0], [0 0],"Color","red","LineWidth",1)
plot3([0 0], [0 3], [0 0],"Color","green","LineWidth",1)
plot3([0 0], [0 0], [0 3],"Color","blue","LineWidth",1)
plot3(DRCSV(:,2),DRCSV(:,3),DRCSV(:,4),"k--")

xlabel("X (m)")
ylabel("Y (m)");
zlabel("Z (m)")
title("Nuvem de pontos obtida utilizando a trajetória pelo Dead-Reckoning")
grid on

