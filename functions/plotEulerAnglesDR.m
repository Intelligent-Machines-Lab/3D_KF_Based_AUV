function plotEulerAnglesDR(DR,gt_data)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    is_sim = 0;
    
    if ~isempty(gt_data)
        is_sim = 1;
    end

    color_std = [0 0.4470 0.7410];
    
    h = figure('Name','Orientação Dead Reckoning');
    
    fig1 = subplot(3,1,1);
    title("Ângulo de rolamento (Eixo X)")
    TIME = DR(:,1);
    data = rad2deg(DR(:,8));

    hold on;
    plot(TIME,data,'Color',color_std);
    if is_sim
        plot(seconds(gt_data.Time),rad2deg(gt_data.data(:,3)),'k:','LineWidth',3);
    end

    grid on;
    xlabel('Tempo (s)');
    ylabel('\phi (°)');
    
    if is_sim
        legend('Valor medido',"Valor verdadeiro" )
    else
        legend('Valor medido')
    end

    fig2 = subplot(3,1,2);
    title("Ângulo de arfagem  (Eixo Y)")
    TIME = DR(:,1);
    data = rad2deg(DR(:,9));

    hold on;
    plot(TIME,data,'Color',color_std);
    if is_sim
        plot(seconds(gt_data.Time),rad2deg(gt_data.data(:,2)),'k:','LineWidth',3);
    end
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('\theta (°)');
    
    if is_sim
        legend('Valor medido',"Valor verdadeiro" )
    else
        legend('Valor medido')
    end
    
    fig3 = subplot(3,1,3);
    title("Ângulo de guinada  (Eixo Z)")
    TIME = DR(:,1);
    data = (DR(:,10));
    
    N = length(data);
    dataPlot = zeros(N,1);
    angle = data(1);
    for ii=2:1:N
        angle = angle + angdiff(data(ii-1),data(ii));
        dataPlot(ii) = angle;   
    end
    data = rad2deg(dataPlot);

    hold on;
    plot(TIME,data,'Color',color_std);
    if is_sim
        plot(seconds(gt_data.Time),rad2deg(gt_data.data(:,1)),'k:','LineWidth',3);
    end
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('\psi (°)');
    
    if is_sim
        legend('Valor medido',"Valor verdadeiro" )
    else
        legend('Valor medido')
    end

    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFig(fig1,path+pasta+"\figs\"+'DR_roll');
        printFig(fig2,path+pasta+"\figs\"+'DR_pitch');
        printFig(fig3,path+pasta+"\figs\"+'DR_yaw');
    
        savefig(path+pasta+"\figs\matlab\"+'DR_eul.fig')
        saveas(h,path+pasta+"\figs\"+'DR_eul.png')
    end
end