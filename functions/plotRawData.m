function plotRawData()
    clc;close all;clear all;
    addpath('./functions')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Load all sensors data
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;
    
    load(path+pasta+"imu_data.mat")
    load(path+pasta+"dvl_data.mat")
    load(path+pasta+"mag_data.mat")
    load(path+pasta+"depth_data.mat")
    load(path+pasta+"p881L_data.mat")
    load(path+pasta+"mag_cal.mat")
    %load(path+pasta+"rpy_data.mat")
    load(path+pasta+"gps_data.mat")
    
    color = [0 0.4470 0.7410];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Getting the size and the period of each array data
    % IMU
    % DVL 
    % MAG 
    % GPS 
    % DEPTH
    % profiling 881L 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    imu_size = size(imu_data,1);
    dvl_size = size(dvl_data,1);
    mag_size = size(mag_data,1);
    depth_size = size(depth_data,1);
    p881L_size = size(p881L_data,1);
    %rpy_size = size(rpy_data,1);
    gps_size = size(gps_data,1);
    
    h = figure('Name','Body Acceleration');
    
    fig1 = subplot(3,1,1);
    
    TIME = seconds(imu_data.Time);
    data = imu_data.IMU_TABLE(:,1);
    plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('a_{bx} (m/s^2)');
    title('Aceleração medida (eixo X) ')
    fig2 = subplot(3,1,2);
    
    TIME = seconds(imu_data.Time);
    data = imu_data.IMU_TABLE(:,2);
    plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('a_{by} (m/s^2)');
    title('Aceleração medida (eixo Y) ')

    fig3 = subplot(3,1,3);
    
    TIME = seconds(imu_data.Time);
    data = imu_data.IMU_TABLE(:,3);
    plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('a_{bz} (m/s^2)');
    title('Aceleração medida (eixo Z) ')
    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFigRaw(fig1,path+pasta+"\figs\"+'acelX_raw');
        printFigRaw(fig2,path+pasta+"\figs\"+'acelY_raw');
        printFigRaw(fig3,path+pasta+"\figs\"+'acelZ_raw');
        
        savefig(path+pasta+"\figs\matlab\"+'accel_raw.fig')
        saveas(h,path+pasta+"\figs\"+'accel_raw.png')
    end

    %%
    h = figure('Name','Body Angular Velocity');
    
    fig1 = subplot(3,1,1);
    
    TIME = seconds(imu_data.Time);
    data = imu_data.IMU_TABLE(:,4);
    plot(TIME,rad2deg(data),'Color',color);
    grid on;xlabel('Time (s)');ylabel('\omega _{bx} (°/s^2)');
    title('Velocidade angular medida (eixo X) ')

    fig2 = subplot(3,1,2);
    
    TIME = seconds(imu_data.Time);
    data = imu_data.IMU_TABLE(:,5);
    plot(TIME,rad2deg(data),'Color',color);
    grid on;xlabel('Time (s)');ylabel('\omega _{by} (°/s^2)');
    title('Velocidade angular medida (eixo Y) ')

    fig3 = subplot(3,1,3);
    
    TIME = seconds(imu_data.Time);
    data = imu_data.IMU_TABLE(:,6);
    plot(TIME,rad2deg(data),'Color',color);
    grid on;xlabel('Time (s)');ylabel('\omega _{bz} (°/s^2)');
    title('Velocidade angular medida (eixo Z) ')
    set(h, 'WindowStyle', 'Docked');
    if saveData
        printFigRaw(fig1,path+pasta+"\figs\"+'gyroX_raw');
        printFigRaw(fig2,path+pasta+"\figs\"+'gyroY_raw');
        printFigRaw(fig3,path+pasta+"\figs\"+'gyroZ_raw');
        
        savefig(path+pasta+"\figs\matlab\"+'gyro_raw.fig')
        saveas(h,path+pasta+"\figs\"+'gyro_raw.png')
    end
    %%
    
    h = figure('Name','Body Velocity');
    
    fig1 = subplot(3,1,1);
    TIME = seconds(dvl_data.Time);
    data = dvl_data.DVL_TABLE(:,1);
    plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('v _{bx} (m/s)');
    title('Velocidade medida (eixo X) ')

    fig2 = subplot(3,1,2);
    TIME = seconds(dvl_data.Time);
    data = dvl_data.DVL_TABLE(:,2);
    plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('v _{by} (m/s)');
    title('Velocidade medida (eixo Y) ')

    fig3 = subplot(3,1,3);
    TIME = seconds(dvl_data.Time);
    data = dvl_data.DVL_TABLE(:,3);
    plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('v _{bz} (m/s)');
    title('Velocidade medida (eixo Z) ')
    set(h, 'WindowStyle', 'Docked');

    if saveData
        printFigRaw(fig1,path+pasta+"\figs\"+'velX_raw');
        printFigRaw(fig2,path+pasta+"\figs\"+'velY_raw');
        printFigRaw(fig3,path+pasta+"\figs\"+'velZ_raw');
        
        savefig(path+pasta+"\figs\matlab\"+'vel_raw.fig')
        saveas(h,path+pasta+"\figs\"+'vel_raw.png')    
    end
    %%
    h = figure('Name','Body Magnetometer');
    
    fig1 = subplot(2,2,1);
    TIME = seconds(mag_data.Time);
    data = mag_data.MAG_TABLE(:,1);
    plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('m _{bx} ()');
    title('Intensidade do campo magnético (eixo X) ')

    fig2 = subplot(2,2,2);
    TIME = seconds(mag_data.Time);
    data = mag_data.MAG_TABLE(:,2);
    plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('m _{by} ()');
    title('Intensidade do campo magnético (eixo Y) ')
    fig3 = subplot(2,2,3);
    TIME = seconds(mag_data.Time);
    data = mag_data.MAG_TABLE(:,3);
    plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('m _{bz} ()');
    title('Intensidade do campo magnético (eixo Z) ')
    
    fig4 = subplot(2,2,4);
    TIME = seconds(mag_data.Time);
    dataMX = mag_data.MAG_TABLE(:,1);
    dataMY = mag_data.MAG_TABLE(:,2);
    dataMZ = mag_data.MAG_TABLE(:,3);
    plot3(dataMX,dataMY,dataMZ,'Color',color,'Marker','*',LineStyle='none');
    grid on;xlabel('MX');ylabel('MY');zlabel('MZ');
    title('Intensidade do campo magnético (eixo XYZ) ')
    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFigRaw(fig1,path+pasta+"\figs\"+'magX_raw');
        printFigRaw(fig2,path+pasta+"\figs\"+'magY_raw');
        printFigRaw(fig3,path+pasta+"\figs\"+'magZ_raw');
        printFigRaw(fig4,path+pasta+"\figs\"+'magXYZ_raw');
    
        savefig(path+pasta+"\figs\matlab\"+'mag_raw.fig')
        saveas(h,path+pasta+"\figs\"+'mag_raw.png')
    end
    %%
    h = figure('Name','Magnetometer Calibration');
    TIME     = seconds(mag_data.Time);
    MAG_BIAS  = MAG_CALIBRATION.BIAS;
    MAG_SCALE = MAG_CALIBRATION.SCALE;
    
    dataMX = (mag_data.MAG_TABLE(:,1) - MAG_BIAS(1))/MAG_SCALE(1);
    dataMY = (mag_data.MAG_TABLE(:,2) - MAG_BIAS(2))/MAG_SCALE(2);
    dataMZ = (mag_data.MAG_TABLE(:,3) - MAG_BIAS(3))/MAG_SCALE(3);
    plot3(dataMX,dataMY,dataMZ,'Color',color,'Marker','*',LineStyle='none');
    grid on;xlabel('MX');ylabel('MY');zlabel('MZ');axis equal;
    title('Dados do magnetómetro com compensação de bias e fator de escala')
    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        savefig(path+pasta+"\figs\matlab\"+'mag_cal_raw.fig')
        saveas(h,path+pasta+"\figs\"+'mag_cal_raw.png')
    end

    %%
    h = figure('Name','Depth position');
    
    TIME = seconds(depth_data.Time);
    data = depth_data.PRESSURE_TABLE(:,1);
    fig1 = plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('p _{z} (m)');
    title('Profundidade medida')

    set(h, 'WindowStyle', 'Docked');
    if saveData
        savefig(path+pasta+"\figs\matlab\"+'depth_raw.fig')
        saveas(h,path+pasta+"\figs\"+'depth_raw.png')
    end

    %%
    h = figure('Name','GPS');
    
    fig1 = subplot(3,1,1);
    TIME = seconds(gps_data.Time);
    data = gps_data.gps(:,1);
    plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('X (m)');
    title('GPS medido (eixo X - Leste)')
    
    fig2 = subplot(3,1,2);
    TIME = seconds(gps_data.Time);
    data = gps_data.gps(:,2);
    plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('Y (m)');
    title('GPS medido (eixo Y - Norte)')

    fig3 = subplot(3,1,3);
    TIME = seconds(gps_data.Time);
    data = gps_data.gps(:,3);
    plot(TIME,data,'Color',color);
    grid on;xlabel('Time (s)');ylabel('Z (m)');
    title('GPS medido (eixo Z - Para cima)')
    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFigRaw(fig1,path+pasta+"\figs\"+'gpsX_raw');
        printFigRaw(fig2,path+pasta+"\figs\"+'gpsY_raw');
        printFigRaw(fig3,path+pasta+"\figs\"+'gpsZ_raw');
        
        savefig(path+pasta+"\figs\matlab\"+'gps_raw.fig')
        saveas(h,path+pasta+"\figs\"+'gps_raw.png')
    end
    %%
    h = figure('Name','Profiling Sonar');
    fig1 = subplot(2,1,1);
    TIME = seconds(p881L_data.Time);
    range = double(p881L_data.Range(1,1));
    data = double(max(p881L_data.Intensities'))*500/range;
    plot(TIME,data,'Color',color,'Marker','.',LineStyle='none');
    grid on;xlabel('Time (s)');ylabel('Distância medida (m)');
    title('Intensidade medida - sensor de mapeamento')

    fig2 = subplot(2,1,2);
    TIME = seconds(p881L_data.Time);
    data = wrapTo2Pi(p881L_data.Angle); 
    plot(TIME,rad2deg(data),'Color',color,'Marker','.',LineStyle='none');
    grid on;xlabel('Time (s)');ylabel('\theta (°)');
    title('Ângulo medido - sensor de mapeamento')
    set(h, 'WindowStyle', 'Docked');
    
    if saveData
        printFigRaw(fig1,path+pasta+"\figs\"+'p881L_Range_raw');
        printFigRaw(fig2,path+pasta+"\figs\"+'p881L_angle_raw');
        
        savefig(path+pasta+"\figs\matlab\"+'p881L_raw.fig')
        saveas(h,path+pasta+"\figs\"+'p881L_raw.png')
    end
end