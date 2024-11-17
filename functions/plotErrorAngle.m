function plotErrorAngle(errorKF,errorDR)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    color_std = [0 0.4470 0.7410];
    color_kf = [1 0 0];
    
    h = figure('Name','Erro de Angle');
    fig1 = subplot(3,1,1);
    title("Erro de rolagem (°)")
    hold on
    N_RMS = 100;
    

    rms_DR = calculateRMS(errorDR(:,2), N_RMS);
    rms_KF = calculateRMS(errorKF(:,2), N_RMS);
    TRMS_DR = 1:1:length(rms_DR);
    TRMS_KF = 1:1:length(rms_KF);
        
    %plot(errorDR(:,1),errorDR(:,2),'Color',color_std,"LineStyle","--");
    %plot(errorKF(:,1),errorKF(:,2),'Color',color_kf,'LineStyle',':');
    plot(TRMS_DR,rms_DR,'Color',color_std);
    plot(TRMS_KF,rms_KF,'Color',color_kf);

    grid on;xlabel('Número do trecho');ylabel('RMS \Delta \phi (°)');
    legend("Dead-Reackoning",'Filtro de Kalman')
    set(h, 'WindowStyle', 'Docked');
    
    


    fig2 = subplot(3,1,2);
    title("Erro de arfagem (pitch)")
    hold on
    %plot(errorDR(:,1),errorDR(:,3),'Color',color_std);
    %plot(errorKF(:,1),errorKF(:,3),'Color',color_kf);
    rms_DR = calculateRMS(errorDR(:,3), N_RMS);
    rms_KF = calculateRMS(errorKF(:,3), N_RMS);
    TRMS_DR = 1:1:length(rms_DR);
    TRMS_KF = 1:1:length(rms_KF);
    plot(TRMS_DR,rms_DR,'Color',color_std);
    plot(TRMS_KF,rms_KF,'Color',color_kf);

    grid on;xlabel('Número do trecho');ylabel('RMS \Delta \theta (°)');
    legend("Dead-Reackoning",'Filtro de Kalman')
    set(h, 'WindowStyle', 'Docked');
    
    fig3 = subplot(3,1,3);
    title("Erro de guinada (yaw)")
    hold on
    %plot(errorDR(:,1),errorDR(:,4),'Color',color_std);
    %plot(errorKF(:,1),errorKF(:,4),'Color',color_kf);
    rms_DR = calculateRMS(errorDR(:,4), N_RMS);
    rms_KF = calculateRMS(errorKF(:,4), N_RMS);
    TRMS_DR = 1:1:length(rms_DR);
    TRMS_KF = 1:1:length(rms_KF);
    plot(TRMS_DR,rms_DR,'Color',color_std);
    plot(TRMS_KF,rms_KF,'Color',color_kf);
    
    grid on;xlabel('Número do trecho');ylabel('RMS \Delta \psi (°)');
    legend("Dead-Reackoning",'Filtro de Kalman')
    set(h, 'WindowStyle', 'Docked');
    
end