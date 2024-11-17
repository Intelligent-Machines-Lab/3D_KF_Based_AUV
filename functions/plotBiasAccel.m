function plotBiasAccel(KFCSV_XE,KFCSV_XED,gt_data)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    is_sim = 0;
    
    if ~isempty(gt_data)
        is_sim = 1;
    end    
    
    color_std = [0 0.4470 0.7410];
    
    h = figure('Name','Acelerômetro bias');
    
    fig1 = subplot(3,1,1);
    title("Acelerômetro bias (Eixo X)")
    TIME = KFCSV_XE(:,1);
    data = (KFCSV_XE(:,11));
    data_std  = (KFCSV_XED(:,11));
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
    ylabel('bias_{ax} (m/s^2)');
    
    if is_sim
        legend('Bias estimado',"Bias verdadeiro",'3 desvios padrão' )
    else
        legend('Bias estimado','3 desvios padrão' )
    end
    
    fig2 = subplot(3,1,2);
    title("Acelerômetro bias (Eixo Y)")
    TIME = KFCSV_XE(:,1);
    data = (KFCSV_XE(:,12));
    data_std  = (KFCSV_XED(:,12));
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
    xlabel('Tempo (s)');ylabel('bias_{ay} (m/s^2)');
    
    if is_sim
        legend('Bias estimado',"Bias verdadeiro",'3 desvios padrão' )
    else
        legend('Bias estimado','3 desvios padrão' )
    end

    fig3 = subplot(3,1,3);
    title("Acelerômetro bias (Eixo Z)")
    TIME = KFCSV_XE(:,1);
    data = (KFCSV_XE(:,13));
    data_std  = (KFCSV_XED(:,13));
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
    xlabel('Tempo (s)');ylabel('bias_{az} (m/s^2)');
    
    if is_sim
        legend('Bias estimado',"Bias verdadeiro",'3 desvios padrão' )
    else
        legend('Bias estimado','3 desvios padrão' )
    end

    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFig(fig1,path+pasta+"\figs\"+'KF_bias_ax');
        printFig(fig2,path+pasta+"\figs\"+'KF_bias_ay');
        printFig(fig3,path+pasta+"\figs\"+'KF_bias_az');
    
        savefig(path+pasta+"\figs\matlab\"+'KF_bias_accel.fig')
        saveas(h,path+pasta+"\figs\"+'KF_bias_accel.png')
    end
    
end