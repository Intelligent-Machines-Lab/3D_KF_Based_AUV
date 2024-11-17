function plotVelocity(KFCSV_XE,KFCSV_XED,gt_data)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    is_sim = 0;
    
    if ~isempty(gt_data)
        is_sim = 1;
    end

    color_std = [0 0.4470 0.7410];
    color_kf = [1 0 0];

    h = figure('Name','Velocidade');
    
    fig1 = subplot(3,1,1);
    title("Velocidade no eixo X")
    TIME = KFCSV_XE(:,1);
    data = KFCSV_XE(:,2);
    data_std  = KFCSV_XED(:,2);
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'Color',color_kf );
    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,1),'k:','LineWidth',3);
    end
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    grid on;
    xlabel('Tempo (s)');
    ylabel('v_{x} (m/s)');
    
    if is_sim
        legend('Valor estimado',"Valor verdadeiro",'3 desvios padrão' )
    else
        legend('Valor estimado','3 desvios padrão' )
    end

    fig2 = subplot(3,1,2);
    title("Velocidade no eixo Y")
    TIME = KFCSV_XE(:,1);
    data = KFCSV_XE(:,3);
    data_std  = KFCSV_XED(:,3);
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'Color',color_kf );
    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,2),'k:','LineWidth',3);
    end

    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('v_{y} (m/s)');
    
    if is_sim
        legend('Valor estimado',"Valor verdadeiro",'3 desvios padrão' )
    else
        legend('Valor estimado','3 desvios padrão' )
    end

    fig3 = subplot(3,1,3);
    title("Velocidade no eixo Z")
    TIME = KFCSV_XE(:,1);
    data = KFCSV_XE(:,4);
    data_std  = KFCSV_XED(:,4);
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'Color',color_kf );
    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,3),'k:','LineWidth',3);
    end
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('v_{z} (m/s)');
    
    if is_sim
        legend('Valor estimado',"Valor verdadeiro",'3 desvios padrão')
    else
        legend('Valor estimado','3 desvios padrão' )
    end

    set(h, 'WindowStyle', 'Docked');
    
    if saveData

        printFig(fig1,path+pasta+"\figs\"+'KF_vx');
        printFig(fig2,path+pasta+"\figs\"+'KF_vy');
        printFig(fig3,path+pasta+"\figs\"+'KF_vz');
        
        savefig(path+pasta+"\figs\matlab\"+'KF_vel.fig')
        saveas(h,path+pasta+"\figs\"+'KF_vel.png')
    end
end