function plot_KF_DR_GT_Position(KF,DR,gt_data)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    is_sim = 0;
    
    if ~isempty(gt_data)
        is_sim = 1;
    end
    
    color_std = [0 0.4470 0.7410];
    
    h = figure('Name','2D Posição');
    
    fig1 = subplot(2,2,1);
    title("Posição no eixo X")
    TIME = KF(:,1);
    data = KF(:,5);
    

    
    hold on;
    plot(TIME,data,"r");
    
    TIME = DR(:,1);
    data = DR(:,2);
    plot(TIME,data,"b");
    
    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,1),'k:','LineWidth',3);
    end

    grid on;
    xlabel('Tempo (s)');
    ylabel('p_{x} (m)');
    
    if is_sim
        legend('Posição EKF',"Posição DR",'Posição verdadeira' )
    else
        legend('Posição EKF',"Posição DR" )
    end
    
    fig2 = subplot(2,2,2);
    title("Posição no eixo Y")
    TIME = KF(:,1);
    data = KF(:,6);

    hold on;
    plot(TIME,data,'r');

    TIME = DR(:,1);
    data = DR(:,3);
    plot(TIME,data,'b');

    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,2),'k:','LineWidth',3);
    end
    
    grid on;
    xlabel('Tempo (s)');ylabel('p_{y} (m)');

    if is_sim
        legend('Posição EKF',"Posição DR",'Posição verdadeira' )
    else
        legend('Posição EKF',"Posição DR" )
    end
    
    fig3 = subplot(2,2,3);
    title("Posição no eixo Z")
    TIME = KF(:,1);
    data = KF(:,7);
  
    hold on;
    plot(TIME,data,'r');

    TIME = DR(:,1);
    data = DR(:,4);
    plot(TIME,data,'b');
    
    if is_sim
        plot(seconds(gt_data.Time),gt_data.data(:,3),'k:','LineWidth',3);
    end
 
    grid on;
    xlabel('Tempo (s)');ylabel('p_{z} (m)');
    
    if is_sim
        legend('Posição EKF',"Posição DR",'Posição verdadeira' )
    else
        legend('Posição EKF',"Posição DR" )
    end

    set(h, 'WindowStyle', 'Docked');


    fig4 = subplot(2,2,4);
    title("Posição 2D")
    TIME = KF(:,1);
    data = KF(:,7);
 
    hold on;
    plot(KF(:,5),KF(:,6),'r');
    plot(DR(:,2),DR(:,3),'b');
    if is_sim
        plot(gt_data.data(:,1),gt_data.data(:,2),'k:','LineWidth',3);
    end
 
    grid on;
    xlabel('p_{x} (m)');ylabel('p_{y} (m)');
    axis equal

    if is_sim
        legend('Posição EKF',"Posição DR",'Posição verdadeira' )
    else
        legend('Posição EKF',"Posição DR" )
    end

    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFig(fig1,path+pasta+"\figs\"+'KFDR_px');
        printFig(fig2,path+pasta+"\figs\"+'KFDR_py');
        printFig(fig3,path+pasta+"\figs\"+'KFDR_pz');
        printFig(fig3,path+pasta+"\figs\"+'KFDR_pxy');
    
        savefig(path+pasta+"\figs\matlab\"+'KFDR_pos.fig')
        saveas(h,path+pasta+"\figs\"+'KFDR_pos.png')
    end
    
end