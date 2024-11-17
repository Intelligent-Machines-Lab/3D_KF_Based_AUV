function plotErrorPosition(errorKF,errorDR)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    color_std = [0 0.4470 0.7410];
    color_kf = [1 0 0];
    
    h = figure('Name','Erro de posição');
    fig1 = subplot(2,2,1);
    title("Erro de posição em X")
    hold on
    N_RMS = 100;
    rms_DR = calculateRMS(errorDR(:,2), N_RMS);
    rms_KF = calculateRMS(errorKF(:,2), N_RMS);
    TRMS_DR = 1:1:length(rms_DR);
    TRMS_KF = 1:1:length(rms_KF);
    
    disp(length(rms_DR))
    disp(length(rms_KF))

    disp(length(errorDR))
    disp(length(errorKF))
    
    %plot(errorDR(:,1),errorDR(:,2),'Color',color_std,"LineStyle","--");
    %plot(errorKF(:,1),errorKF(:,2),'Color',color_kf,'LineStyle',':');
    plot(TRMS_DR,rms_DR,'Color',color_std);
    plot(TRMS_KF,rms_KF,'Color',color_kf);

    grid on;xlabel('Número do trecho');ylabel('RMS \Delta x (m)');
    legend("Dead-Reackoning",'Filtro de Kalman')
    set(h, 'WindowStyle', 'Docked');
    
    


    fig2 = subplot(2,2,2);
    title("Erro de posição em Y")
    hold on
    %plot(errorDR(:,1),errorDR(:,3),'Color',color_std);
    %plot(errorKF(:,1),errorKF(:,3),'Color',color_kf);
    rms_DR = calculateRMS(errorDR(:,3), N_RMS);
    rms_KF = calculateRMS(errorKF(:,3), N_RMS);
    TRMS_DR = 1:1:length(rms_DR);
    TRMS_KF = 1:1:length(rms_KF);
    plot(TRMS_DR,rms_DR,'Color',color_std);
    plot(TRMS_KF,rms_KF,'Color',color_kf);

    grid on;xlabel('Número do trecho');ylabel('RMS \Delta y (m)');
    legend("Dead-Reackoning",'Filtro de Kalman')
    set(h, 'WindowStyle', 'Docked');
    
    fig3 = subplot(2,2,3);
    title("Erro de posição em Z")
    hold on
    %plot(errorDR(:,1),errorDR(:,4),'Color',color_std);
    %plot(errorKF(:,1),errorKF(:,4),'Color',color_kf);
    rms_DR = calculateRMS(errorDR(:,4), N_RMS);
    rms_KF = calculateRMS(errorKF(:,4), N_RMS);
    TRMS_DR = 1:1:length(rms_DR);
    TRMS_KF = 1:1:length(rms_KF);
    plot(TRMS_DR,rms_DR,'Color',color_std);
    plot(TRMS_KF,rms_KF,'Color',color_kf);
    
    grid on;xlabel('Número do trecho');ylabel('RMS \Delta z (m)');
    legend("Dead-Reackoning",'Filtro de Kalman')
    set(h, 'WindowStyle', 'Docked');
    
    
    fig4 = subplot(2,2,4);
    
    title("")
    hold on
    %plot(errorDR(:,1),errorDR(:,5),'Color',color_std);
    %plot(errorKF(:,1),errorKF(:,5),'Color',color_kf);
    rms_DR = calculateRMS(errorDR(:,5), N_RMS);
    rms_KF = calculateRMS(errorKF(:,5), N_RMS);
    TRMS_DR = 1:1:length(rms_DR);
    TRMS_KF = 1:1:length(rms_KF);
    plot(TRMS_DR,rms_DR,'Color',color_std);
    plot(TRMS_KF,rms_KF,'Color',color_kf);
    
    title("Erro de posição em XYZ")
    grid on;xlabel('Número do trecho');ylabel('RMS \Delta xyz (m)');
    legend("Dead-Reackoning",'Filtro de Kalman')
    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFig(fig1,path+pasta+"\figs\"+'errorX');
        printFig(fig2,path+pasta+"\figs\"+'errorY');
        printFig(fig3,path+pasta+"\figs\"+'errorZ');
        printFig(fig4,path+pasta+"\figs\"+'errorXYZ');
    
        savefig(path+pasta+"\figs\matlab\"+'KF_error_pos.fig')
        saveas(h,path+pasta+"\figs\"+'KF_error_pos.png')
    end
end