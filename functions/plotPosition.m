function plotPosition(KFCSV_XE,KFCSV_XED,gt_data)
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
    
    h = figure('Name','Posição');
    
    fig1 = subplot(3,1,1);
    title("Posição no eixo X")
    TIME = KFCSV_XE(:,1);
    data = KFCSV_XE(:,5);
    data_std  = KFCSV_XED(:,5);
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'Color',color_kf,'LineWidth',2);
    
    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,1),'k:','LineWidth',2);
    end
    plot(TIME,plus_std,'Color',color_std,'LineWidth',2);
    plot(TIME,minus_std,'Color',color_std,'LineWidth',2);
    grid on;
    xlabel('Tempo (s)');
    ylabel('p_{x} (m)');
    
    if is_sim
        legend('Valor estimado',"Valor verdadeiro",'3 desvios padrão' )
        %legend('Valor estimado',"Valor verdadeiro")
    else
        legend('Valor estimado','3 desvios padrão')
    end
    
    fig2 = subplot(3,1,2);
    title("Posição no eixo Y")
    TIME = KFCSV_XE(:,1);
    data = KFCSV_XE(:,6);
    data_std  = KFCSV_XED(:,6);
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
    xlabel('Tempo (s)');ylabel('p_{y} (m)');
    if is_sim
        legend('Valor estimado',"Valor verdadeiro",'3 desvios padrão' )
    else
        legend('Valor estimado','3 desvios padrão' )
    end
    
    fig3 = subplot(3,1,3);
    title("Posição no eixo Z")
    TIME = KFCSV_XE(:,1);
    data = KFCSV_XE(:,7);
    data_std  = KFCSV_XED(:,7);
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
    xlabel('Tempo (s)');ylabel('p_{z} (m)');
    
    if is_sim
        legend('Valor estimado',"Valor verdadeiro",'3 desvios padrão' )
    else
        legend('Valor estimado','3 desvios padrão' )
    end
    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFig(fig1,path+pasta+"\figs\"+'KF_px');
        printFig(fig2,path+pasta+"\figs\"+'KF_py');
        printFig(fig3,path+pasta+"\figs\"+'KF_pz');
    
        savefig(path+pasta+"\figs\matlab\"+'KF_pos.fig')
        saveas(h,path+pasta+"\figs\"+'KF_pos.png')
    end

    h = figure('Name','Posição2');
    
    title("Posição no eixo X")
    TIME = KFCSV_XE(:,1);
    data = KFCSV_XE(:,5);
    data_std  = KFCSV_XED(:,5);
    plus_std  =  3*data_std;
    minus_std = -3*data_std;
    
    hold on;
    
    
   

    if is_sim
         errorKF = errorTrajXYZ([seconds(gt_data.Time) gt_data.data],[KFCSV_XE(:,1) KFCSV_XE(:,5:7)]);
        %plot(seconds(gt_data.Time),gt_data.data(:,1),'k:','LineWidth',3);
        plot(errorKF(:,1),errorKF(:,2),'r');
        plot(TIME,plus_std,'Color',color_std);
        plot(TIME,minus_std,'Color',color_std);
        grid on;
        xlabel('Tempo (s)');
        ylabel('p_{x} (m)');
        legend('Erro em p_x','3 desvios padrão' )
        set(h, 'WindowStyle', 'Docked');
    end
    


    

end