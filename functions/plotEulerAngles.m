function plotEulerAngles(KFCSV_XE,KFCSV_XED,gt_data)
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

    h = figure('Name','Orientação');
    
    fig1 = subplot(3,1,1);
    title("Ângulo de rolamento (Eixo X)")
    TIME = KFCSV_XE(:,1);
    data = rad2deg(KFCSV_XE(:,8));
    data_std  = rad2deg(KFCSV_XED(:,8));
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'Color',color_kf);

    if is_sim
        plot(seconds(gt_data.Time),rad2deg(gt_data.data(:,3)),'k:','LineWidth',3);
    end

    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    grid on;
    xlabel('Tempo (s)');
    ylabel('\phi (°)');
    
    if is_sim
        legend('Valor estimado',"Valor verdadeiro",'3 desvios padrão' )
    else
        legend('Valor estimado','3 desvios padrão' )
    end
    
    fig2 = subplot(3,1,2);
    title("Ângulo de arfagem  (Eixo Y)")
    TIME = KFCSV_XE(:,1);
    data = rad2deg(KFCSV_XE(:,9));
    data_std  = rad2deg(KFCSV_XED(:,9));
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'Color',color_kf);

        
    if is_sim
        plot(seconds(gt_data.Time),rad2deg(gt_data.data(:,2)),'k:','LineWidth',3);
    end
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    
    grid on;
    xlabel('Tempo (s)');ylabel('\theta (°)');
    
    if is_sim
        legend('Valor estimado',"Valor verdadeiro",'3 desvios padrão' )
    else
        legend('Valor estimado','3 desvios padrão' )
    end

    fig3 = subplot(3,1,3);
    title("Ângulo de guinada  (Eixo Z)")
    TIME = KFCSV_XE(:,1);
    data = KFCSV_XE(:,10);

    N = length(data);
    dataPlot = zeros(N,1);
    angle = data(1);
    for ii=2:1:N
        angle = angle + angdiff(data(ii-1),data(ii));
        dataPlot(ii) = angle;   
    end
    data = rad2deg(dataPlot);
    data_std  = rad2deg(KFCSV_XED(:,10));
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    
  
    plot(TIME,data,'Color',color_kf);

    if is_sim
        plot(seconds(gt_data.Time),rad2deg(gt_data.data(:,1)),'k:','LineWidth',3);
    end
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    
    grid on;
    xlabel('Tempo (s)');ylabel('\psi (°)');
    if is_sim
        legend('Valor estimado',"Valor verdadeiro",'3 desvios padrão' )
    else
        legend('Valor estimado','3 desvios padrão' )
    end

    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFig(fig1,path+pasta+"\figs\"+'KF_roll');
        printFig(fig2,path+pasta+"\figs\"+'KF_pitch');
        printFig(fig3,path+pasta+"\figs\"+'KF_yaw');
    
        savefig(path+pasta+"\figs\matlab\"+'KF_eul.fig')
        saveas(h,path+pasta+"\figs\"+'KF_eul.png')
    end
end