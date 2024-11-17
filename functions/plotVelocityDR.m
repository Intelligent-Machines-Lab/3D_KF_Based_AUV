function plotVelocityDR(DR,gt_data)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    is_sim = 0;
    
    if ~isempty(gt_data)
        is_sim = 1;
    end

    color_std = [0 0.4470 0.7410];
    
    h = figure('Name','Velocidade Dead Reckoning');
    
    fig1 = subplot(3,1,1);
    title("Velocidade no eixo X")
    TIME = DR(:,1);
    data = DR(:,5);

    hold on;
    plot(TIME,data,'Color',color_std);
    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,1),'k:','LineWidth',3);
    end

    grid on;
    xlabel('Tempo (s)');
    ylabel('v_{x} (m/s)');
    
    if is_sim
        legend('Valor medido',"Valor verdadeiro" )
    else
        legend('Valor medido')
    end

    fig2 = subplot(3,1,2);
    title("Velocidade no eixo Y")
    TIME = DR(:,1);
    data = DR(:,6);

    hold on;
    plot(TIME,data,'Color',color_std);
    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,2),'k:','LineWidth',3);
    end
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('v_{y} (m/s)');
    
    if is_sim
        legend('Valor medido',"Valor verdadeiro" )
    else
        legend('Valor medido')
    end
    
    fig3 = subplot(3,1,3);
    title("Velocidade no eixo Z")
    TIME = DR(:,1);
    data = DR(:,7);
    
    hold on;
    plot(TIME,data,'Color',color_std);
    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,3),'k:','LineWidth',3);
    end
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('v_{z} (m/s)');
    
    if is_sim
        legend('Valor medido',"Valor verdadeiro" )
    else
        legend('Valor medido')
    end

    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFig(fig1,path+pasta+"\figs\"+'DR_vx');
        printFig(fig2,path+pasta+"\figs\"+'DR_vy');
        printFig(fig3,path+pasta+"\figs\"+'DR_vz');
    
        savefig(path+pasta+"\figs\matlab\"+'DR_vel.fig')
        saveas(h,path+pasta+"\figs\"+'DR_vel.png')
    end
end