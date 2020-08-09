%Clear and Set working Directory.
clear all; clc; close all;
cd 'C:\Users\Oliver\Documents\GitHub\IMU_Motion_Tracker'
addpath('Raw_Data')

%% Read In IMU Data
%Define Measurement Units and Sample Rates
Accel_Unit = 1;
RolPitYaw_Unit = 'Rad';
IMU_Fs = 100;

%Calib G Vector 
Calib_G_N = [0.7868,0.2575 ,9.7691];
Calib_G_B = [0.4589,-0.0347 ,9.8642];

%Read In Amout
Read_In_Start = 500;
Read_In_End = 2001;
Data_Length = Read_In_End - Read_In_Start;

Data_Temp = readmatrix('IMU_3.csv');
RolPitYaw_b = Data_Temp(Read_In_Start:Read_In_End ,2:4);
RolPitYaw_dt_b = Data_Temp(Read_In_Start:Read_In_End ,5:7);
Accel_b = Data_Temp(Read_In_Start:Read_In_End ,8:10)*Accel_Unit;
Magnet_b = Data_Temp(Read_In_Start:Read_In_End ,11:13);
time_b = Data_Temp(Read_In_Start:Read_In_End ,1)/IMU_Fs;

%% Intial Data Cleaning and Preprossecing. 
for i = 1:Data_Length
    if RolPitYaw_Unit == 'Rad'
        RolPitYaw_b(i,:) =  deg2rad(RolPitYaw_b(i));
        RolPitYaw_dt_b(i,:) = deg2rad(RolPitYaw_dt_b(i));
    end
    Q_RolPitYaw_b(i) = quaternion(RolPitYaw_b(i,:),'rotvecd');
    Q_RolPitYaw_dt_b(i) = quaternion(RolPitYaw_dt_b(i,:),'rotvecd');
end


%% Define our AHRS filter.
decim = 2;
Accel_Noise = 0.00019247;
Gyro_Noise = 9.1385e-5;
Gyro_Drift = 3.0462e-13;
Lin_Accel_Noise = 0.0096236;

time_n = time_b(1:decim:end,:);

fuse = ahrsfilter('SampleRate',IMU_Fs,'DecimationFactor',decim, ...
    'AccelerometerNoise',Accel_Noise,'GyroscopeNoise',Gyro_Noise, ...
    'GyroscopeDriftNoise',Gyro_Drift,'LinearAccelerationNoise',Lin_Accel_Noise);

Q_RolPitYaw_n = fuse(Accel_b,RolPitYaw_dt_b,Magnet_b);


%% Plot Orientation Estimate results
figure('Name','Orientation Estimate')
plot(time_n,eulerd(Q_RolPitYaw_n,'ZYX','frame'))
title('Orientation Estimate')
legend('X-axis', 'Y-axis', 'Z-axis')
xlabel('Time (s)')
ylabel('Rotation (degrees)')

%% Pre-prosscesing of the acceleration signal.
%Filter_Design 
Order = 30;
fc = 20;
fs = IMU_Fs;

%Butterworth for noise filtering. 
[b,a] = butter(Order,fc/(fs/2));
But_Accel_b = filter(b,a, Accel_b);

%Calibration Stage to negate Gravity
Calib_Steps = 500;



%Calib_Z = mean(But_Accel_b(1:Calib_Steps,:))
%Calib_Z = [x_g_b y_g_b z_g_b]
%Mag_Calib_Z = sqrt(sum(Calib_Z.^2)) 
%Calib_Orient_Z = meanrot(Q_RolPitYaw_b(1:Calib_Steps))


for i = 1:length(time_n)
    Accel_n = rotatepoint(conj(Q_RolPitYaw_n(i)),But_Accel_b(1:decim:end,:));
    %Accel_n = Accel_n - Calib_G_N;    
end

for i = 1:length(time_b)
    %Subtract rotatepoint(conj(Calib_G_B(i)),But_Accel_b(1:decim:end,:))
    But_Accel_b(i,:) = But_Accel_b(i,:) - Calib_G_B;
end

% %% Plot Acceleration Estimate results
% figure('Name','Acceleration Estimate (Navigation Frame)')
% plot(time_n(Read_In_Start:end-400),Accel_n(Read_In_Start:end-400,:))
% title('Acceleration Estimate (Navigation Frame)')
% legend('X-axis', 'Y-axis', 'Z-axis')
% xlabel('Time (s)')
% ylabel('z(m)')

figure('Name','Acceleration Estimate (Body Frame)')
plot(time_b(1:end),But_Accel_b(1:end,:))
title('Acceleration Estimate (Body Frame)')
legend('X-axis', 'Y-axis', 'Z-axis')
xlabel('Time (s)')
ylabel('z(ms^-2)')


%% FFT On Acceleration
Fs = fs;            % Sampling frequency                    
T = 1/Fs;             % Sampling period  
Fn = Fs/2;           %Nyquist Frequency
L = length(time_n);             % Length of signal
t = (0:L-1)*T;        % Time vector

X = 4;

Y = fft(But_Accel_b);

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
Iv = 1:length(P1);
IvX = 1:X;
FX = f(X);

plot(f,P1) 
figure('Name','Single-Sided Amplitude Spectrum of X(t)')
plot(f,P1)
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')

FLen = 48;                                                          % Discrete Filter Order
b_filt = fir1(FLen, FX/Fn);   

figure('Name','Filter Responce')
freqz(b_filt, 1, 4096, Fs)

data_filtered = fftfilt(b_filt,But_Accel_b);
figure('Name','Filtered Acceleration (b Frame)')
title('Post Acceleration Estimate (Body Frame)')
legend('X-axis', 'Y-axis', 'Z-axis')
xlabel('Time (s)')
ylabel('z(ms^-2)')
plot(time_b,data_filtered)

%% Signal Integration and Detrending
But_Accel_b = detrend(data_filtered);
Int_But_Vel_B = zeros(length(time_n),3);
for i = 1:length(time_n)
    Int_But_Vel_B(i,:) = trapz(decim/IMU_Fs,data_filtered(1:i,:));
end

Int_But_Vel_B_Detrend = detrend(Int_But_Vel_B);

%% %% Plot Velocity Estimate Results
figure('Name','Velocity Estimate (Detrended) (Body Frame)')
plot(time_n(Read_In_Start:end-400),Int_But_Vel_B_Detrend(Read_In_Start:end-400,:))
title('Velocity Estimate (Body Frame)')
legend('X-axis', 'Y-axis', 'Z-axis')
xlabel('Time (s)')
ylabel('z(m)')

figure('Name','Velocity Estimate Body Frame)')
plot(time_n(Read_In_Start:end-400),Int_But_Vel_B(Read_In_Start:end-400,:))
title('Velocity Estimate (Body Frame)')
legend('X-axis', 'Y-axis', 'Z-axis')
xlabel('Time (s)')
ylabel('z(m)')


Int_But_Vel_B_Detrend2 = detrend(Int_But_Vel_B_Detrend);

figure('Name','Velocity Estimate (Detrendedx2)Body Frame)')
plot(time_n(Read_In_Start:end-400),Int_But_Vel_B_Detrend2(Read_In_Start:end-400,:))
title('Velocity Estimate (Body Frame)')
legend('X-axis', 'Y-axis', 'Z-axis')
xlabel('Time (s)')
ylabel('z(m)')

%% Calculate Position
Int_But_Pos_B = zeros(length(time_n),3);
for i = 1:length(time_n)
    Int_But_Pos_B(i,:) = trapz(decim/IMU_Fs,Int_But_Vel_B_Detrend(1:i,:));
end

Int_But_Pos_B = detrend(detrend(Int_But_Pos_B))
Position_z = detrend(Int_But_Pos_B(:,2))

%% Plot Position
figure('Name','Position Estimate (Detrended)Body Frame)')
plot(time_n(Read_In_Start:end),Int_But_Pos_B(Read_In_Start:end,:))
title('Position Estimate (Body Frame)')
legend('X-axis', 'Y-axis', 'Z-axis')
xlabel('Time (s)')
ylabel('z(m)')


figure('Name','Position Estimate')
plot(time_n(Read_In_Start:end-400),Position_z(Read_In_Start:end-400,:))



