%% MULTIPLE IMU EKF 
% Author: Laura Train
% Date of the last update Apr 21 2021
%
% Implement an EKF on the IMU configuration for a known motion

%% Include paths
matlabrc
clear; clc; close all;

addpath ../synthetic-data/simulation/imu2cm
addpath ../synthetic-data/simulation/
addpath ../ins/
addpath ../conversions/
addpath ../kalman/
addpath ../data-acquire/data-processing/post-processing/
addpath ../data-acquire/data-processing/post-processing/files/
addpath ../data-acquire/data-processing/calibration


%% read data

test = csvread('kalman6.csv');

[imu1, imu2, imu3, imu4] = read_realdata(test);


%% SIMULATE MOTION FOR THE BODY (CENTER OF MASS)

% Initial considerations:
% B, body reference frame of the each of the IMUs
% V, body reference frame of the center of mass of the multi-IMU body
% DCMvb is the DCM from the ith-imu body reference frame to the center of
% mass of body. It is fixed and known a priori. 
% imuMAIN, the synthetic motion of the center of mass

vec_times = [length(imu1.t), length(imu2.t), length(imu3.t), length(imu4.t)];
[N, index] = min(vec_times);

if index == 1
    imuMAIN.t = imu1.t;
elseif index == 2
    imuMAIN.t = imu2.t;
elseif index == 3
    imuMAIN.t = imu3.t;
elseif index == 4 
    imuMAIN.t = imu4.t;
end

imuMAIN.ini_align = [0, 0, 0];

% Generate synthetic motion by defining angular velocities
w = 0;
imuMAIN.wv(:,1) = w*ones(N,1);
imuMAIN.wv(:,2) = zeros(N, 1);
imuMAIN.wv(:,3) = zeros(N,1);

% Generate synthetic motion for the acceleration and magnetic field.
[imuMAIN] = IMU_simulator(imuMAIN);


%% choose skew or in-plane configuration

config = input('Skew or in-plane configuration? \n Enter 1 for skew, enter 0 for in-plane: ');

if config == 1
    
    imu1.roll0 = 0;
    imu1.pitch0 = 0;
    imu1.yaw0 = 0;
    
    imu2.roll0 = 0;
    imu2.pitch0 = 120;
    imu2.yaw0 = 45;
    
    imu3.roll0 = 0;
    imu3.pitch0 = 120;
    imu3.yaw0 = 180;
    
    imu4.roll0 = 0;
    imu4.pitch0 = 120;
    imu4.yaw0 = -45;
    
    % length from the center of mass to each IMU
    L = 0.2;
    
elseif config == 0
    
    imu1.roll0 = 180;
    imu1.pitch0 = 0;
    imu1.yaw0 = 0;
    
    imu2.roll0 = 180;
    imu2.pitch0 = 0;
    imu2.yaw0 = 0;
    
    imu3.roll0 = 180;
    imu3.pitch0 = 0;
    imu3.yaw0 = 0;
    
    imu4.roll0 = 180;
    imu4.pitch0 = 0;
    imu4.yaw0 = 0;
    
    % length from the center of mass to each IMU
    L = 0;
else
    error('Invalid input. You must enter 1 or 0')
end


%%
calibrate = input('Do you want to calibrate the IMUs? Enter 1 if yes, enter 0 if not: ');
if calibrate == 1
    [imu1_raw, imu2_raw, imu3_raw, imu4_raw] = read_4imu(test);


    [imu1_raw] = calibrate_imu(imu1_raw);
    [imu2_raw] = calibrate_imu(imu2_raw);
    [imu3_raw] = calibrate_imu(imu3_raw);
    [imu4_raw] = calibrate_imu(imu4_raw);


    [imu1_error] = store_errors(imu1_raw);
    [imu2_error] = store_errors(imu2_raw);
    [imu3_error] = store_errors(imu3_raw);
    [imu4_error] = store_errors(imu4_raw);

end

%% GENERATE DATA FOR IMU NUMBER 1
% imu1, imu located at a point different from the center of mass
% imu1MAINcomputed, the data for the center of mass obtained from the IMU1

load imu1_error.mat

% relative orientation of the IMU1 with respect to the center of mass of
% the body
imu1.ini_align = [0, 0, 0];
imu1.ini_align_err = deg2rad([0.5, 0.5, 0.5]);


imu1.m_std = [0.005, 0.005, 0.005];

% Assign errors and reference matrices
[imu1] = set_imu(L, imu1, imu1_error);

% remove offset bias error
% imu1.wb(:,1) = imu1.wb(:,1) - imu1_error.gyroX.offset;
% imu1.wb(:,2) = imu1.wb(:,2) - imu1_error.gyroY.offset;
% imu1.wb(:,3) = imu1.wb(:,3) - imu1_error.gyroZ.offset;
% 
% imu1.fb(:,1) = imu1.fb(:,1) - imu1_error.accX.offset;
% imu1.fb(:,2) = imu1.fb(:,2) - imu1_error.accY.offset;
% imu1.fb(:,3) = imu1.fb(:,3) - imu1_error.accZ.offset;


% convert the synthetic data of the body into data for the IMU1
[imu1_synth] = VIMU_to_IMU(imuMAIN, imu1);

% add noise to the imu sensor
imu1_synth.wb = imu1_synth.wb(1:N,:) + imu1.g_std.*randn(N,3);
imu1_synth.fb = imu1_synth.fb(1:N,:) + imu1.a_std.*randn(N,3);
imu1_synth.mb = imu1_synth.mb(1:N,:) + imu1.m_std.*randn(N,3);

%% GENERATE DATA FOR IMU NUMBER 2
% imu2, imu located at a point different from the center of mass
% imu2MAINcomputed, the data for the center of mass obtained from the IMU2

load imu2_error.mat

% relative orientation of the IMU1 with respect to the center of mass of
% the body
imu2.ini_align = [0, 0, 0];
imu2.ini_align_err = deg2rad([0.5, 0.5, 0.5]);


imu2.m_std = [0.5, 0.5, 0.5];

% Assign errors and reference matrices
[imu2] = set_imu(L, imu2, imu2_error);

% remove offset bias error
% imu2.wb(:,1) = imu2.wb(:,1) - imu2_error.gyroX.offset;
% imu2.wb(:,2) = imu2.wb(:,2) - imu2_error.gyroY.offset;
% imu2.wb(:,3) = imu2.wb(:,3) - imu2_error.gyroZ.offset;
% 
% imu2.fb(:,1) = imu2.fb(:,1) - imu2_error.accX.offset;
% imu2.fb(:,2) = imu2.fb(:,2) - imu2_error.accY.offset;
% imu2.fb(:,3) = imu2.fb(:,3) - imu2_error.accZ.offset;

% convert the synthetic data of the body into data for the IMU1
[imu2_synth] = VIMU_to_IMU(imuMAIN, imu2);

% add noise to the imu sensor
imu2_synth.wb = imu2_synth.wb(1:N,:) + imu2.g_std.*randn(N,3);
imu2_synth.fb = imu2_synth.fb(1:N,:) + imu2.a_std.*randn(N,3);
imu2_synth.mb = imu2_synth.mb(1:N,:) + imu2.m_std.*randn(N,3);


%% GENERATE DATA FOR IMU NUMBER 3
% imu3, imu located at a point different from the center of mass
% imu3MAINcomputed, the data for the center of mass obtained from the IMU3

load imu3_error.mat

% relative orientation of the IMU3 with respect to the center of mass of
% the body
imu3.ini_align = [0, 0, 0];
imu3.ini_align_err = deg2rad([0.5, 0.5, 0.5]);


imu3.m_std = [0.005, 0.005, 0.005];

% Assign errors and reference matrices
[imu3] = set_imu(L, imu3, imu3_error);

% remove offset bias error
% imu3.wb(:,1) = imu3.wb(:,1) - imu3_error.gyroX.offset;
% imu3.wb(:,2) = imu3.wb(:,2) - imu3_error.gyroY.offset;
% imu3.wb(:,3) = imu3.wb(:,3) - imu3_error.gyroZ.offset;
% 
% imu3.fb(:,1) = imu3.fb(:,1) - imu3_error.accX.offset;
% imu3.fb(:,2) = imu3.fb(:,2) - imu3_error.accY.offset;
% imu3.fb(:,3) = imu3.fb(:,3) - imu3_error.accZ.offset;

% convert the synthetic data of the body into data for the IMU1
[imu3_synth] = VIMU_to_IMU(imuMAIN, imu1);

% add noise to the imu sensor
imu3_synth.wb = imu3_synth.wb(1:N,:) + imu3.g_std.*randn(N,3);
imu3_synth.fb = imu3_synth.fb(1:N,:) + imu3.a_std.*randn(N,3);
imu3_synth.mb = imu3_synth.mb(1:N,:) + imu3.m_std.*randn(N,3);

%% GENERATE DATA FOR IMU NUMBER 4
% imu4, imu located at a point different from the center of mass
% imu4MAINcomputed, the data for the center of mass obtained from the IMU4

load imu4_error.mat

% relative orientation of the IMU4 with respect to the center of mass of
% the body
imu4.ini_align = [0, 0, 0];
imu4.ini_align_err = deg2rad([0.5, 0.5, 0.5]);


imu4.m_std = [0.005, 0.005, 0.005];

% Assign errors and reference matrices
[imu4] = set_imu(L, imu4, imu4_error);

% remove offset bias error
% imu4.wb(:,1) = imu4.wb(:,1) - imu4_error.gyroX.offset;
% imu4.wb(:,2) = imu4.wb(:,2) - imu4_error.gyroY.offset;
% imu4.wb(:,3) = imu4.wb(:,3) - imu4_error.gyroZ.offset;
% 
% imu4.fb(:,1) = imu4.fb(:,1) - imu4_error.accX.offset;
% imu4.fb(:,2) = imu4.fb(:,2) - imu4_error.accY.offset;
% imu4.fb(:,3) = imu4.fb(:,3) - imu4_error.accZ.offset;

% convert the synthetic data of the body into data for the IMU1
[imu4_synth] = VIMU_to_IMU(imuMAIN, imu1);

% add noise to the imu sensor
imu4_synth.wb = imu4_synth.wb(1:N,:) + imu4.g_std.*randn(N,3);
imu4_synth.fb = imu4_synth.fb(1:N,:) + imu4.a_std.*randn(N,3);
imu4_synth.mb = imu4_synth.mb(1:N,:) + imu4.m_std.*randn(N,3);

%% DATA FUSION
% [navCM] = arch4_imu2cm_filter(imu1, imu2, imu3, imu4);

%% PLOT RESULTS

imuMAIN.ini_align = deg2rad([0, 0, 0]);
[quat, euler] = attitude_computer(imuMAIN);
euler = rad2deg(euler);

% Plot comparison between true attitude vs EKF output
figure(1)
plot(navCM.t, quat(:,1), 'r', navCM.t, quat(:,2), 'b', navCM.t, quat(:,3), 'g', navCM.t, quat(:,4), 'k', ...
    navCM.t, navCM.qua1(:,1), 'or', navCM.t, navCM.qua1(:,2), 'ob', navCM.t, navCM.qua1(:,3), 'og', navCM.t, navCM.qua1(:,4), 'ok')
xlabel('Time [s]')
ylabel('quaternions')
legend('q1', 'q2,', 'q3', 'q4', 'q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions. IMU 1.')
legend('location','southwest')


figure(2)
plot(navCM.t, euler(:,1), 'r', navCM.t, euler(:,2), 'b', navCM.t, euler(:,3), 'g', ...
     navCM.t, navCM.roll1, 'or', navCM.t, navCM.pitch1, 'ob', navCM.t, navCM.yaw1, 'og')
xlabel('Time [s]')
ylabel('Euler angles [deg]')
legend('roll', 'pitch,', 'yaw', 'roll Kalman','pitch Kalman','yaw Kalman')
grid minor
title('Euler angles. IMU 1.')
legend('location','southwest')


figure(3)
plot(navCM.t, quat(:,1), 'r', navCM.t, quat(:,2), 'b', navCM.t, quat(:,3), 'g', navCM.t, quat(:,4), 'k', ...
    navCM.t, navCM.qua2(:,1), 'or', navCM.t, navCM.qua2(:,2), 'ob', navCM.t, navCM.qua2(:,3), 'og', navCM.t, navCM.qua2(:,4), 'ok')
xlabel('Time [s]')
ylabel('quaternions')
legend('q1', 'q2,', 'q3', 'q4', 'q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions. IMU 2.')
legend('location','southwest')


figure(4)
plot(navCM.t, euler(:,1), 'r', navCM.t, euler(:,2), 'b', navCM.t, euler(:,3), 'g', ...
     navCM.t, navCM.roll2, 'or', navCM.t, navCM.pitch2, 'ob', navCM.t, navCM.yaw2, 'og')
xlabel('Time [s]')
ylabel('Euler angles [deg]')
legend('roll', 'pitch,', 'yaw', 'roll Kalman','pitch Kalman','yaw Kalman')
grid minor
title('Euler angles. IMU 2.')
legend('location','southwest')


figure(5)
plot(navCM.t, quat(:,1), 'r', navCM.t, quat(:,2), 'b', navCM.t, quat(:,3), 'g', navCM.t, quat(:,4), 'k', ...
    navCM.t, navCM.qua3(:,1), 'or', navCM.t, navCM.qua3(:,2), 'ob', navCM.t, navCM.qua3(:,3), 'og', navCM.t, navCM.qua3(:,4), 'ok')
xlabel('Time [s]')
ylabel('quaternions')
legend('q1', 'q2,', 'q3', 'q4', 'q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions. IMU 3.')
legend('location','southwest')


figure(6)
plot(navCM.t, euler(:,1), 'r', navCM.t, euler(:,2), 'b', navCM.t, euler(:,3), 'g', ...
     navCM.t, navCM.roll3, 'or', navCM.t, navCM.pitch3, 'ob', navCM.t, navCM.yaw3, 'og')
xlabel('Time [s]')
ylabel('Euler angles [deg]')
legend('roll', 'pitch,', 'yaw', 'roll Kalman','pitch Kalman','yaw Kalman')
grid minor
title('Euler angles. IMU 3.')
legend('location','southwest')


figure(7)
plot(navCM.t, quat(:,1), 'r', navCM.t, quat(:,2), 'b', navCM.t, quat(:,3), 'g', navCM.t, quat(:,4), 'k', ...
    navCM.t, navCM.qua4(:,1), 'or', navCM.t, navCM.qua4(:,2), 'ob', navCM.t, navCM.qua4(:,3), 'og', navCM.t, navCM.qua4(:,4), 'ok')
xlabel('Time [s]')
ylabel('quaternions')
legend('q1', 'q2,', 'q3', 'q4', 'q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions. IMU 4.')
legend('location','southwest')


figure(8)
plot(navCM.t, euler(:,1), 'r', navCM.t, euler(:,2), 'b', navCM.t, euler(:,3), 'g', ...
     navCM.t, navCM.roll4, 'or', navCM.t, navCM.pitch4, 'ob', navCM.t, navCM.yaw4, 'og')
xlabel('Time [s]')
ylabel('Euler angles [deg]')
legend('roll', 'pitch,', 'yaw', 'roll Kalman','pitch Kalman','yaw Kalman')
grid minor
title('Euler angles. IMU 4.')
legend('location','southwest')


%% PLOT RAW DATA

figure(9)
plot(imu1.t, imu1.fb(:,1), 'r', imu2.t, imu2.fb(:,1), 'b', imu3.t, imu3.fb(:,1), 'g', imu4.t, imu4.fb(:,1), 'k')
title("Accelerometer X coordinate [$m/s^2$]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Acceleration [$m/s^2$]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best")
grid minor

figure(10)
plot(imu1.t, imu1.fb(:,2), 'r', imu2.t, imu2.fb(:,2), 'b', imu3.t, imu3.fb(:,2), 'g', imu4.t, imu4.fb(:,2), 'k')
title("Accelerometer Y coordinate [$m/s^2$]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Acceleration [$m/s^2$]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best")
grid minor

figure(11)
plot(imu1.t, imu1.fb(:,3), 'r', imu2.t, imu2.fb(:,3), 'b', imu3.t, imu3.fb(:,3), 'g', imu4.t, imu4.fb(:,3), 'k')
title("Accelerometer Z coordinate [$m/s^2$]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Acceleration [$m/s^2$]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best")
grid minor


figure(12)
plot(imu1.t, imu1.wb(:,1), 'r', imu2.t, imu2.wb(:,1), 'b', imu3.t, imu3.wb(:,1), 'g', imu4.t, imu4.wb(:,1), 'k')
title("Gyroscope X coordinate [rad/s]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Angular velocity [rad/s]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best")
grid minor

figure(13)
plot(imu1.t, imu1.wb(:,2), 'r', imu2.t, imu2.wb(:,2), 'b', imu3.t, imu3.wb(:,2), 'g', imu4.t, imu4.wb(:,2), 'k')
title("Gyroscope Y coordinate [rad/s]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Angular velocity [rad/s]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best")
grid minor

figure(14)
plot(imu1.t, imu1.wb(:,3), 'r', imu2.t, imu2.wb(:,3), 'b', imu3.t, imu3.wb(:,3), 'g', imu4.t, imu4.wb(:,3), 'k')
title("Gyroscope Z coordinate [rad/s]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Angular velocity [rad/s]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best")
grid minor