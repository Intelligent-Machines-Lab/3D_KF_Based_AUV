function plotInovationEulerAngles(Ik,STD_Ik)
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;

    color_std = [0 0.4470 0.7410];
    
    h = figure('Name','Inovação Euler');
    
    fig1 = subplot(3,1,1);
    title("Inovação ângulo de rolamento (eixo X)")
    TIME = Ik(:,1);
    data = rad2deg(Ik(:,8));
    data_std  = rad2deg(STD_Ik(:,8));
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'r');
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    grid on;
    xlabel('Tempo (s)');
    ylabel('I_{\phi} (°)');
    
    legend('Inovação','3 desvios padrão' )

    fig2 = subplot(3,1,2);
    title("Inovação ângulo de arfagem (eixo Y)")
    TIME = Ik(:,1);
    data = rad2deg(Ik(:,9));
    data_std  = rad2deg(STD_Ik(:,9));
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'r');
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('I_{\theta} (°)');
    legend('Inovação','3 desvios padrão' )

    fig3 = subplot(3,1,3);
    title("Inovação ângulo de guinada (eixo Z)")
    TIME = Ik(:,1);
    data = rad2deg(Ik(:,10));
    data_std  = rad2deg(STD_Ik(:,10));
    plus_std  = data+3*data_std;
    minus_std = data-3*data_std;
    
    hold on;
    plot(TIME,data,'r');
    plot(TIME,plus_std,'Color',color_std);
    plot(TIME,minus_std,'Color',color_std);
    
    grid on;
    
    xlabel('Tempo (s)');ylabel('I_{\psi} (°)');
    legend('Inovação','3 desvios padrão' )

    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFig(fig1,path+pasta+"\figs\"+'KF_iroll');
        printFig(fig2,path+pasta+"\figs\"+'KF_ipitch');
        printFig(fig3,path+pasta+"\figs\"+'KF_iyaw');
    
        savefig(path+pasta+"\figs\matlab\"+'KF_ieul.fig')
        saveas(h,path+pasta+"\figs\"+'KF_ieul.png')
    end
end