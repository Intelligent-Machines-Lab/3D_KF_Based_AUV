clear all;close all;clc;
load("MAIN.mat")
path = "";
pasta = MAIN.folder;

load(path+pasta+"mag_data.mat")
%load(path+pasta+"mag_cal.mat")
load(path+pasta+"rpy_data.mat")
load(path+pasta+"imu_data.mat")



% Sample data
data = mag_data.MAG_TABLE; % Dataset 1


data = data - mean(data); % Center the data
[Aeye,beye,expMFSeye] = magcal(data,'sym')

BIAS = beye';
SCALE = diag(Aeye);
MX = (data(:,1) - BIAS(1))*SCALE(1)/expMFSeye;
MY = (data(:,2) - BIAS(2))*SCALE(2)/expMFSeye;
MZ = (data(:,3) - BIAS(3))*SCALE(3)/expMFSeye;

figure
plot3(MX,MY,MZ)
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
axis equal

MAG_CALIBRATION.BIAS   = BIAS;
MAG_CALIBRATION.SCALE  = SCALE;
MAG_CALIBRATION.DEC    = -0.374373124552784;
save('mag_cal.mat','MAG_CALIBRATION');


N = length(data);
heading = zeros(N,1);

for ii=1:1:N
    mx = MX(ii);
    my = MY(ii);
    
    heading(ii,1) = rad2deg(getHeading(mx,my));
end

t = 1:1:N;
hold on
dec = rad2deg(-0.3744)*ones(1,N);
figure
hold on
plot(t,dec)
plot(t,heading)
figure
plot(seconds(rpy_data.Time),rad2deg(rpy_data.RPY_TABLE(:,1)))
grid on
 return