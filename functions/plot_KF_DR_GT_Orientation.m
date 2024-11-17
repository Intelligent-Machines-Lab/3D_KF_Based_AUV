function plot_KF_DR_GT_Orientation(KF,DR,gt_data)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    is_sim = 0;
    
    if ~isempty(gt_data)
        is_sim = 1;
    end
    
    color_std = [0 0.4470 0.7410];
    
    h = figure('Name','Orientação');
    
    fig1 = subplot(3,1,1);
    title("Ângulo de rolamento (Eixo X)")
    TIME = KF(:,1);
    data = rad2deg(KF(:,8));
    

    
    hold on;
    plot(TIME,data,"r");
    
    TIME = DR(:,1);
    data = rad2deg(DR(:,8));
    plot(TIME,data,"b");
    
    %if is_sim
    %    plot(seconds(gt_data.Time),gt_data.data(:,1),'k:','LineWidth',3);
    %end

    grid on;
    xlabel('Tempo (s)');
    ylabel('\phi (°)');
    
    %if is_sim
        legend('Orientação EKF',"Orientação DR")
    %else
    %    legend('Posição EKF',"Posição DR" )
    %end
    
    fig2 = subplot(3,1,2);
    title("Ângulo de arfagem (Eixo Y)")
    TIME = KF(:,1);
    data = rad2deg(KF(:,9));

    hold on;
    plot(TIME,data,'r');

    TIME = DR(:,1);
    data = rad2deg(DR(:,9));
    plot(TIME,data,'b');

    %if is_sim
    %    plot(seconds(gt_data.Time),gt_data.data(:,2),'k:','LineWidth',3);
    %end
    
    grid on;
    xlabel('Tempo (s)');
    ylabel('\theta (°)');

    legend('Orientação EKF',"Orientação DR")
    %if is_sim
    %    legend('Posição EKF',"Posição DR",'Posição verdadeira' )
    %else
    %    legend('Posição EKF',"Posição DR" )
    %end
    
    fig3 = subplot(3,1,3);
    title("Ângulo de guinada (Eixo Z)")
    TIME = KF(:,1);
    data = rad2deg(KF(:,10));
  
    hold on;
    plot(TIME,data,'r');

    TIME = DR(:,1);
    data = rad2deg(DR(:,10));
    plot(TIME,data,'b');
    
    %if is_sim
    %    plot(seconds(gt_data.Time),gt_data.data(:,3),'k:','LineWidth',3);
    %end
 
    grid on;
    %xlabel('Tempo (s)');ylabel('p_{z} (m)');
    
    %if is_sim
    %    legend('Posição EKF',"Posição DR",'Posição verdadeira' )
    %else
    %    legend('Posição EKF',"Posição DR" )
    %end
    xlabel('Tempo (s)');
    ylabel('\theta (°)');

    legend('Orientação EKF',"Orientação DR")

    set(h, 'WindowStyle', 'Docked');


    % fig4 = subplot(2,2,4);
    % title("Posição 2D")
    % TIME = KF(:,1);
    % data = KF(:,7);
    % 
    % hold on;
    % plot(KF(:,5),KF(:,6),'r');
    % plot(DR(:,2),DR(:,3),'b');
    % if is_sim
    %     plot(gt_data.data(:,1),gt_data.data(:,2),'k:','LineWidth',3);
    % end
    % 
    % grid on;
    % xlabel('p_{x} (m)');ylabel('p_{y} (m)');
    % axis equal
    % 
    % if is_sim
    %     legend('Posição EKF',"Posição DR",'Posição verdadeira' )
    % else
    %     legend('Posição EKF',"Posição DR" )
    % end
    % 
    % set(h, 'WindowStyle', 'Docked');
    % 
    % if saveData
    %     printFig(fig1,path+pasta+"\figs\"+'KFDR_px');
    %     printFig(fig2,path+pasta+"\figs\"+'KFDR_py');
    %     printFig(fig3,path+pasta+"\figs\"+'KFDR_pz');
    %     printFig(fig3,path+pasta+"\figs\"+'KFDR_pxy');
    % 
    %     savefig(path+pasta+"\figs\matlab\"+'KFDR_pos.fig')
    %     saveas(h,path+pasta+"\figs\"+'KFDR_pos.png')
    % end
    
end