function patchPointCloud()
    clc;close all;clear all;
    addpath('./functions')
    load("MAIN.mat")
    path = "";
    pasta = MAIN.folder;
    saveData = MAIN.saveData;
    
    load(path+pasta+"DRCSV.mat")
    load(path+pasta+"KFCSV.mat")
    
    %intPt = 0;
    %endPt = 0;
    %KFCSV = KFCSV(intPt:1:endPt,:);

    
    %%

    BEAM_WIDTH_1 = 0.0244;%1.4º
    BEAM_WIDTH_2 = 0.0367;% 2.1
    BEAM_WIDTH_3 = 0.0419;% 2.4º
    
    BEAM_WIDTH = BEAM_WIDTH_3;
    BEAM_INC   = 10;
    
    
    idx = 1;
    DISTRIBUTION = 1;
    dtheta = [];
    dphi = [];
    if DISTRIBUTION == 1
        dtheta   = pi/2 - BEAM_WIDTH/2:BEAM_WIDTH/BEAM_INC:pi/2  + BEAM_WIDTH/2;
        dphi     = -BEAM_WIDTH/2:BEAM_WIDTH/BEAM_INC:BEAM_WIDTH/2;
    else
        mu = [pi/2 0];
        Sigma = [BEAM_WIDTH 0; 0 BEAM_WIDTH];
        Sigma = Sigma*Sigma;
        mv = mvnrnd(mu,Sigma,BEAM_INC);
        dphi   = mv(:,2);
        dtheta = mv(:,1);
    end
    
    n_points = length(dtheta)*length(dphi)*length(KFCSV);
    points = zeros(n_points,6);
    
    KF_points = zeros(length(KFCSV),6);
    DR_points = zeros(length(DRCSV),6);
    
    for ii = 1:1:length(KFCSV)
        x     = KFCSV(ii,5);
        y     = KFCSV(ii,6);
        z     = KFCSV(ii,7);
        px    = KFCSV(ii,11);
        py    = KFCSV(ii,12);
        pz    = KFCSV(ii,13);
    
        normal = [px py pz]' - [x y z]';
                
        if(norm(normal)) > 0
            normal = normal/norm(normal);
        end

        KF_points(ii,:) = [px py pz normal'];
    end
    
    for ii = 1:1:length(DRCSV)
        x     = DRCSV(ii,2);
        y     = DRCSV(ii,3);
        z     = DRCSV(ii,4);
        px    = DRCSV(ii,11);
        py    = DRCSV(ii,12);
        pz    = DRCSV(ii,13);
    
        normal = [px py pz]' - [x y z]';
                
        if(norm(normal)) > 0
            normal = normal/norm(normal);
        end
    
        DR_points(ii,:) = [[px py pz] normal'];
    end
    
    
    
    for ii = 1:1:length(KFCSV)
        p     = KFCSV(ii,14);
        theta = KFCSV(ii,15);
        
        x     = KFCSV(ii,5);
        y     = KFCSV(ii,6);
        z     = KFCSV(ii,7);
        roll  = KFCSV(ii,8);
        pitch = KFCSV(ii,9);
        yaw   = KFCSV(ii,10);
        Rypr = getRotation(yaw,pitch,roll);
    
        for jj=1:1:length(dtheta)
           for kk=1:1:length(dphi)
                bx = p*cos(dtheta(jj))*sin(dphi(kk)+ theta);
                bz = p*sin(dtheta(jj))*sin(dphi(kk)+ theta);
                by = p*cos(dphi(kk)+ theta);
               
                global_points = [x y z]' + Rypr*[bx by bz]';
                normal = global_points -[x y z]';
                
                if(norm(normal)) > 0
                    normal = normal/norm(normal);
                end
    
                points(idx,:) = [global_points' normal'];
                idx = idx + 1;
            end
        end
        
        disp(ii+" de "+length(KFCSV));
    end
    
    if 1

        csvwrite(path+pasta+'\ptClouds\'+'KF_points_volta.csv', KF_points);
        csvwrite(path+pasta+'\ptClouds\'+'DR_points_volta.csv', DR_points);
        csvwrite(path+pasta+'\ptClouds\'+'KF_points_patch_volta.csv', points);

        %csvwrite(path+pasta+'\ptClouds\'+'KF_points_volta.csv', KF_points(fix(length(KF_points)/2):1:end,:));
        %csvwrite(path+pasta+'\ptClouds\'+'DR_points_volta.csv', DR_points(fix(length(DR_points)/2):1:end,:));
        %csvwrite(path+pasta+'\ptClouds\'+'KF_points_patch_volta.csv', points(fix(length(points)/2):1:end,:));

        %csvwrite(path+pasta+'\ptClouds\'+'KF_points_ida.csv', KF_points(1:1:fix(length(KF_points)/2),:));
        %csvwrite(path+pasta+'\ptClouds\'+'DR_points_ida.csv', DR_points(1:1:fix(length(DR_points)/2),:));
        %csvwrite(path+pasta+'\ptClouds\'+'KF_points_patch_ida.csv', points(1:1:fix(length(points)/2),:));
    end
    ptCloud = pointCloud(points(:,1:3))
    pcviewer(ptCloud);
    
    figure
    t = 1:1:length(KFCSV(:,1));
    plot(t,rad2deg(KFCSV(:,10)));
    xlabel('Time (s)')
    ylabel('Yaw (°)')
    grid on

end