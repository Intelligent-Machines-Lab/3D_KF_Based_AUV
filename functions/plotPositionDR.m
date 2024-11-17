function plotPositionDR(DR,gt_data)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    is_sim = 0;
    
    if ~isempty(gt_data)
        is_sim = 1;
    end

    color_std = [0 0.4470 0.7410];
    
    h = figure('Name','Posição Dead Reckoning');
    
    fig1 = subplot(3,1,1);
    title("Posição no eixo X")
    TIME = DR(:,1);
    data = DR(:,2);

    hold on;
    plot(TIME,data,'Color',color_std);
    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,1),'k:','LineWidth',3);
    end

    grid on;
    xlabel('Tempo (s)');
    ylabel('p_{x} (m)');
    
    if is_sim
        legend('Valor medido',"Valor verdadeiro" )
    else
        legend('Valor medido')
    end

    fig2 = subplot(3,1,2);
    title("Posição no eixo Y")
    TIME = DR(:,1);
    data = DR(:,3);

    hold on;
    plot(TIME,data,'Color',color_std);
    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,2),'k:','LineWidth',3);
    end
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('p_{y} (m)');
    
    if is_sim
        legend('Valor medido',"Valor verdadeiro" )
    else
        legend('Valor medido')
    end
    
    fig3 = subplot(3,1,3);
    title("Posição no eixo Z")
    TIME = DR(:,1);
    data = DR(:,4);
    
    hold on;
    plot(TIME,data,'Color',color_std);
    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,3),'k:','LineWidth',3);
    end
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('p_{z} (m)');
    
    if is_sim
        legend('Valor medido',"Valor verdadeiro" )
    else
        legend('Valor medido')
    end

    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFig(fig1,path+pasta+"\figs\"+'DR_px');
        printFig(fig2,path+pasta+"\figs\"+'DR_py');
        printFig(fig3,path+pasta+"\figs\"+'DR_pz');
    
        savefig(path+pasta+"\figs\matlab\"+'DR_pos.fig')
        saveas(h,path+pasta+"\figs\"+'DR_pos.png')
    end
end