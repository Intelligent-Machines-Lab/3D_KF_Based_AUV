function plotInovationVelocity(Ik,STD_Ik)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    color_std = [0 0.4470 0.7410];
    
    h = figure('Name','Inovação das Velocidades');
    
    fig1 = subplot(3,1,1);
    title("Inovação velocidade no eixo X")
    TIME = Ik(:,1);
    data = Ik(:,2);
    data_std  = STD_Ik(:,2);
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'r');
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    grid on;
    xlabel('Tempo (s)');
    ylabel('I_{v_x} (m/s)');
    
    legend('Inovação','3 desvios padrão' )

    fig2 = subplot(3,1,2);
    title("Inovação velocidade no eixo Y")
    TIME = Ik(:,1);
    data = Ik(:,3);
    data_std  = STD_Ik(:,3);
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'r');
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('I_{v_y} (m/s)');
    legend('Inovação','3 desvios padrão' )

    fig3 = subplot(3,1,3);
    title("Inovação velocidade no eixo Z")
    TIME = Ik(:,1);
    data = Ik(:,4);
    data_std  = STD_Ik(:,4);
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'r');
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('I_{v_z} (m/s)');
    legend('Inovação','3 desvios padrão' )

    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFig(fig1,path+pasta+"\figs\"+'KF_ivx');
        printFig(fig2,path+pasta+"\figs\"+'KF_ivy');
        printFig(fig3,path+pasta+"\figs\"+'KF_ivz');
    
        savefig(path+pasta+"\figs\matlab\"+'KF_ivel.fig')
        saveas(h,path+pasta+"\figs\"+'KF_ivel.png')
    end
end