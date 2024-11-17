function plotInovationPosition(Ik,STD_Ik)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    color_std = [0 0.4470 0.7410];
    
    h = figure('Name','Inovação das Posições');
    
    fig1 = subplot(3,1,1);
    title("Inovação posição no eixo X")
    TIME = Ik(:,1);
    data = Ik(:,5);
    data_std  = STD_Ik(:,5);
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'r');
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    grid on;
    xlabel('Tempo (s)');
    ylabel('I_{p_x} (m)');
    
    legend('Inovação','3 desvios padrão',Location='best')

    fig2 = subplot(3,1,2);
    title("Inovação posição no eixo Y")
    TIME = Ik(:,1);
    data = Ik(:,6);
    data_std  = STD_Ik(:,6);
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'r');
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('I_{p_y} (m)');
    legend('Inovação','3 desvios padrão',Location='best')

    fig3 = subplot(3,1,3);
    title("Inovação posição no eixo Z")
    TIME = Ik(:,1);
    data = Ik(:,7);
    data_std  = STD_Ik(:,7);
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'r');
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('I_{p_z} (m)');
    legend('Inovação','3 desvios padrão',Location='best')

    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFig(fig1,path+pasta+"\figs\"+'KF_ipx');
        printFig(fig2,path+pasta+"\figs\"+'KF_ipy');
        printFig(fig3,path+pasta+"\figs\"+'KF_ipz');
    
        savefig(path+pasta+"\figs\matlab\"+'KF_ipos.fig')
        saveas(h,path+pasta+"\figs\"+'KF_ipos.png')
    end
end