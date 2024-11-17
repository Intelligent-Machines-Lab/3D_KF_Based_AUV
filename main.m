%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Author:Juan R. B. F. S.
%3D Kalman Filter Based
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;clear all;close all;
addpath('./functions')

%% Datasets
%dataset = DATASET.REAL_JAGUARI_2;
%dataset = DATASET.REAL_JAGUARI_J2_IDA;
%dataset = DATASET.REAL_JAGUARI_J2_VOLTA;
%dataset  = DATASET.REAL_FOZ_CHAPECO;

%dataset = DATASET.REAL_CAPAO_PRETO;%OK PLOT
%dataset = DATASET.REAL_JAGUARI_1;%OK PLOT
dataset = DATASET.REAL_MONTE_CLARO;%OK PLOT

%dataset  = DATASET.SIM_CAPAO_PRETO; %OK PLOT
%dataset = DATASET.SIM_JAGUARI; %OK PLOT %OK PLOT
%dataset = DATASET.SIM_MONTE_CLARO; %OK PLOT

%run  = RUN.KF;
%run  = RUN.DR;
%run  = RUN.KF_DR;
run  = RUN.PLOT;
%run  = RUN.PLOT_RAW
%run  = RUN.FOLDER;

SAVE_PLOTS = 0;
UPDATA_KF_PARAMS = false;

%REAL
%PARAMS.Q = [0.1,0.1,0.1,0.01,0.01,0.01];
PARAMS.Q = [0.18,0.18,0.18,0.0873,0.0873,0.0873];
PARAMS.R = [0.07,0.07,0.07,0.1,0.1,0.1,deg2rad(2),deg2rad(2),deg2rad(5)];
PARAMS.P = [0.5,0.5,0.5,10,10,10,deg2rad(10),deg2rad(10),deg2rad(10),1,1,1,0.1,0.1,0.1];
PARAMS.betta = [0.001 0.001 0.001 (0.1)^4 (0.1)^4 (0.1)^4 deg2rad(0.1)^2 deg2rad(0.1)^2 deg2rad(0.1)^2  0.001 0.001 0.001 0.001 0.001 0.001];


%SIMULATION CP PRETO
% PARAMS.Q = [0.1 0.1 0.1,0.01,0.01,0.01];
% PARAMS.R = [0.01,0.01,0.01,0.01,0.01,0.1, deg2rad(2),deg2rad(2),deg2rad(5)];
% PARAMS.P = [0.2,0.2,0.2,5,5,1,deg2rad(10),deg2rad(10),deg2rad(10),0.1,0.1,0.1,0.1,0.1,0.1];
% PARAMS.betta = [0.0001 0.0001 0.0001 0.0001 0.0001 0.0001 0.0001 0.0001 0.0001 0.0001 0.0001 0.0001 0.0001 0.0001 0.0001];

 %%
REAL_CAPAO_PRETO      = ".\DATA\REAL_CAPAO_PRETO\";
REAL_JAGUARI_1        = ".\DATA\REAL_JAGUARI_1\";
REAL_JAGUARI_2        = ".\DATA\REAL_JAGUARI_2\";
REAL_MONTE_CLARO      = ".\DATA\REAL_MONTE_CLARO\";
REAL_JAGUARI_J2_IDA   = ".\DATA\REAL_JAGUARI_J2_IDA\";
REAL_JAGUARI_J2_VOLTA = ".\DATA\REAL_JAGUARI_J2_VOLTA\";
REAL_FOZ_CHAPECO      = ".\DATA\REAL_FOZ_CHAPECO\";

SIM_CAPAO_PRETO   = ".\DATA\SIM_CAPAO_PRETO\";
SIM_JAGUARI       = ".\DATA\SIM_JAGUARI\";
SIM_MONTE_CLARO   = ".\DATA\SIM_MONTE_CLARO\";

folder = "";
isGPS  = 0;
is_sim = 0;
switch dataset
    case DATASET.REAL_CAPAO_PRETO
        folder = REAL_CAPAO_PRETO;
        isGPS  = 1;
        is_sim = 0;
    case DATASET.REAL_JAGUARI_1
        folder = REAL_JAGUARI_1;
        isGPS  = 0;
        is_sim = 0;
    case DATASET.REAL_JAGUARI_2
        folder = REAL_JAGUARI_2;
        isGPS  = 0;
        is_sim = 0;
    case DATASET.REAL_MONTE_CLARO
        folder = REAL_MONTE_CLARO;
        isGPS  = 0;
        is_sim = 0;
    case DATASET.SIM_CAPAO_PRETO
        folder = SIM_CAPAO_PRETO;
        isGPS  = 1;
        is_sim = 1;
    case DATASET.SIM_JAGUARI
        folder = SIM_JAGUARI;
        isGPS  = 0;
        is_sim = 1;
    case DATASET.SIM_MONTE_CLARO
        folder = SIM_MONTE_CLARO;
        isGPS  = 0;
        is_sim = 1;
    case DATASET.REAL_JAGUARI_J2_IDA
        folder = REAL_JAGUARI_J2_IDA;
        isGPS  = 0;
        is_sim = 0;
    case DATASET.REAL_JAGUARI_J2_VOLTA
        folder = REAL_JAGUARI_J2_VOLTA;
        isGPS  = 0;
        is_sim = 0;
    case DATASET.REAL_FOZ_CHAPECO
        folder = REAL_FOZ_CHAPECO;
        isGPS  = 1;
        is_sim = 0;
    otherwise
        disp('Folder not Found')
        return
end

MAIN.folder = folder;
MAIN.isGPS  = isGPS;
MAIN.is_sim = is_sim; 
MAIN.DATASET  = dataset;
MAIN.saveData = 0;

if SAVE_PLOTS
    MAIN.saveData = 1;
end    
save('MAIN.mat','MAIN');

%%
if UPDATA_KF_PARAMS
    save(folder+'PARAMS.mat','PARAMS')
end

%%
switch run
    case RUN.KF
        KF_BASED();
        plotData();
    case RUN.DR
        DeadReackoning();
        plotData();
    case RUN.KF_DR
        KF_BASED();
        DeadReackoning();
        plotData();
    case RUN.PLOT
        plotData();
    case RUN.PLOT_RAW
        plotRawData()
    otherwise
        return
end

%patchPointCloud()