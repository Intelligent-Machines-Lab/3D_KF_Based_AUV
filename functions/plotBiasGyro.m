function plotBiasGyro(KFCSV_XE,KFCSV_XED,gt_data)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    is_sim = 0;
    
    if ~isempty(gt_data)
        is_sim = 1;
    end    
    
    color_std = [0 0.4470 0.7410];
    
    h = figure('Name','Giroscópio bias');
    
    fig1 = subplot(3,1,1);
    title("Giroscópio bias (Eixo X)")
    TIME = KFCSV_XE(:,1);
    data = (KFCSV_XE(:,14));
    data_std  = (KFCSV_XED(:,14));
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'r');

    if is_sim
        plot(seconds(gt_data.Time),(gt_data.data(:,1)),'k:','LineWidth',3);
    end

    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    grid on;
    xlabel('Tempo (s)');
    ylabel('bias_{\omega _x} (°/s)');
    
    if is_sim
        legend('Bias estimado',"Bias verdadeiro",'3 desvios padrão' )
    else
        legend('Bias estimado','3 desvios padrão' )
    end
    
    fig2 = subplot(3,1,2);
    title("Giroscópio bias (Eixo Y)")
    TIME = KFCSV_XE(:,1);
    data = (KFCSV_XE(:,15));
    data_std  = (KFCSV_XED(:,15));
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'r');

        
    if is_sim
        plot(seconds(gt_data.Time),(gt_data.data(:,2)),'k:','LineWidth',3);
    end
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    
    grid on;
    xlabel('Tempo (s)');ylabel('bias_{\omega _y} (°/s)');
    
    if is_sim
        legend('Bias estimado',"Bias verdadeiro",'3 desvios padrão' )
    else
        legend('Bias estimado','3 desvios padrão' )
    end

    fig3 = subplot(3,1,3);
    title("Giroscópio bias (Eixo Z)")
    TIME = KFCSV_XE(:,1);
    data = (KFCSV_XE(:,16));
    data_std  = (KFCSV_XED(:,16));
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'r');

    if is_sim
        plot(seconds(gt_data.Time),(gt_data.data(:,3)),'k:','LineWidth',3);
    end
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    
    grid on;
    xlabel('Tempo (s)');ylabel('bias_{\omega _z} (°/s)');
    
    if is_sim
        legend('Bias estimado',"Bias verdadeiro",'3 desvios padrão' )
    else
        legend('Bias estimado','3 desvios padrão' )
    end

    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFig(fig1,path+pasta+"\figs\"+'KF_bias_wx');
        printFig(fig2,path+pasta+"\figs\"+'KF_bias_wy');
        printFig(fig3,path+pasta+"\figs\"+'KF_bias_wz');
    
        savefig(path+pasta+"\figs\matlab\"+'KF_bias_gyro.fig')
        saveas(h,path+pasta+"\figs\"+'KF_bias_gyro.png')
    end
end