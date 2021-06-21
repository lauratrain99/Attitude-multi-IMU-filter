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

%% parse datadata-acquire/data-processing/spatial-manager
system('python ../data-acquire/data-processing/spatial-manager/spatial_parser_file_VerBETA.py')

%% read data

test = csvread('gyrotable2.csv');

[imu1, imu2, imu3, imu4] = read_realdata(test);

load imu1_error.mat
load imu2_error.mat
load imu3_error.mat
load imu4_error.mat

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
    
    imu1.roll0 = 0;
    imu1.pitch0 = 180;
    imu1.yaw0 = 180;
    
    imu2.roll0 = 0;
    imu2.pitch0 = 180;
    imu2.yaw0 = 180;
    
    imu3.roll0 = 0;
    imu3.pitch0 = 180;
    imu3.yaw0 = 180;
    
    imu4.roll0 = 0;
    imu4.pitch0 = 180;
    imu4.yaw0 = 180;
    
    % length from the center of mass to each IMU
    L = 0;
else
    error('Invalid input. You must enter 1 or 0')
end


%%
static = input('Is the IMU static? Enter 1 if yes, enter 0 if not: ');
if static == 1
    [imu1_error.gyroX.offset, imu1_error.gyroX.std] = get_offset(imu1.wb(:,1), 0);
    imu1_error.gyroX.bias_stab = 1e-5;
    [imu1_error.gyroY.offset, imu1_error.gyroY.std] = get_offset(imu1.wb(:,2), 0);
    imu1_error.gyroX.bias_stab = 1e-5;
    [imu1_error.gyroZ.offset, imu1_error.gyroZ.std] = get_offset(imu1.wb(:,3), 0);
    imu1_error.gyroZ.bias_stab = 1e-5;

    [imu1_error.accX.offset, imu1_error.accX.std] = get_offset(imu1.fb(:,1), 0);
    imu1_error.accX.bias_stab = 1e-4;
    [imu1_error.accY.offset, imu1_error.accY.std] = get_offset(imu1.fb(:,2), 0);
    imu1_error.accY.bias_stab = 1e-4;
    [imu1_error.accZ.offset, imu1_error.accZ.std] = get_offset(imu1.fb(:,3), -9.81);
    imu1_error.accZ.bias_stab = 1e-4;
    
    [imu2_error.gyroX.offset, imu2_error.gyroX.std] = get_offset(imu2.wb(:,1), 0);
    imu2_error.gyroX.bias_stab = 1e-5;
    [imu2_error.gyroY.offset, imu2_error.gyroY.std] = get_offset(imu2.wb(:,2), 0);
    imu2_error.gyroY.bias_stab = 1e-5;
    [imu2_error.gyroZ.offset, imu2_error.gyroZ.std] = get_offset(imu2.wb(:,3), 0);
    imu2_error.gyroZ.bias_stab = 1e-5;

    [imu2_error.accX.offset, imu2_error.accX.std] = get_offset(imu2.fb(:,1), 0);
    imu2_error.accX.bias_stab = 1e-4;
    [imu2_error.accY.offset, imu2_error.accY.std] = get_offset(imu2.fb(:,2), 0);
    imu2_error.accY.bias_stab = 1e-4;
    [imu2_error.accZ.offset, imu2_error.accZ.std] = get_offset(imu2.fb(:,3), -9.81);
    imu2_error.accZ.bias_stab = 1e-4;
    
    [imu3_error.gyroX.offset, imu3_error.gyroX.std] = get_offset(imu3.wb(:,1), 0);
    imu3_error.gyroX.bias_stab = 1e-5;
    [imu3_error.gyroY.offset, imu3_error.gyroY.std] = get_offset(imu3.wb(:,2), 0);
    imu3_error.gyroY.bias_stab = 1e-5;
    [imu3_error.gyroZ.offset, imu3_error.gyroZ.std] = get_offset(imu3.wb(:,3), 0);
    imu3_error.gyroZ.bias_stab = 1e-5;

    [imu3_error.accX.offset, imu3_error.accX.std] = get_offset(imu3.fb(:,1), 0);
    imu3_error.accX.bias_stab = 1e-4;
    [imu3_error.accY.offset, imu3_error.accY.std] = get_offset(imu3.fb(:,2), 0);
    imu3_error.accY.bias_stab = 1e-4;
    [imu3_error.accZ.offset, imu3_error.accZ.std] = get_offset(imu3.fb(:,3), -9.81);
    imu3_error.accZ.bias_stab = 1e-4;

    
    [imu4_error.gyroX.offset, imu4_error.gyroX.std] = get_offset(imu4.wb(:,1), 0);
    imu4_error.gyroX.bias_stab = 1e-5;
    [imu4_error.gyroY.offset, imu4_error.gyroY.std] = get_offset(imu4.wb(:,2), 0);
    imu4_error.gyroY.bias_stab = 1e-5;
    [imu4_error.gyroZ.offset, imu4_error.gyroZ.std] = get_offset(imu4.wb(:,3), 0);
    imu4_error.gyroZ.bias_stab = 1e-5;

    [imu4_error.accX.offset, imu4_error.accX.std] = get_offset(imu4.fb(:,1), 0);
    imu4_error.accX.bias_stab = 1e-4;
    [imu4_error.accY.offset, imu4_error.accY.std] = get_offset(imu4.fb(:,2), 0);
    imu4_error.accY.bias_stab = 1e-4;
    [imu4_error.accZ.offset, imu4_error.accZ.std] = get_offset(imu4.fb(:,3), -9.81);
    imu4_error.accZ.bias_stab = 1e-4;

    
    disp("The bias offset was removed from raw values")
else
    disp("The bias offset was not removed from raw values")
end


imu1.wb(:,1) = imu1.wb(:,1) - imu1_error.gyroX.offset;
imu1.wb(:,2) = imu1.wb(:,2) - imu1_error.gyroY.offset;
imu1.wb(:,3) = imu1.wb(:,3) - imu1_error.gyroZ.offset;

imu1.fb(:,1) = imu1.fb(:,1) - imu1_error.accX.offset;
imu1.fb(:,2) = imu1.fb(:,2) - imu1_error.accY.offset;
imu1.fb(:,3) = imu1.fb(:,3) - imu1_error.accZ.offset;

imu2.wb(:,1) = imu2.wb(:,1) - imu2_error.gyroX.offset;
imu2.wb(:,2) = imu2.wb(:,2) - imu2_error.gyroY.offset;
imu2.wb(:,3) = imu2.wb(:,3) - imu2_error.gyroZ.offset;

imu2.fb(:,1) = imu2.fb(:,1) - imu2_error.accX.offset;
imu2.fb(:,2) = imu2.fb(:,2) - imu2_error.accY.offset;
imu2.fb(:,3) = imu2.fb(:,3) - imu2_error.accZ.offset;

imu3.wb(:,1) = imu3.wb(:,1) - imu3_error.gyroX.offset;
imu3.wb(:,2) = imu3.wb(:,2) - imu3_error.gyroY.offset;
imu3.wb(:,3) = imu3.wb(:,3) - imu3_error.gyroZ.offset;

imu3.fb(:,1) = imu3.fb(:,1) - imu3_error.accX.offset;
imu3.fb(:,2) = imu3.fb(:,2) - imu3_error.accY.offset;
imu3.fb(:,3) = imu3.fb(:,3) - imu3_error.accZ.offset;

imu4.wb(:,1) = imu4.wb(:,1) - imu4_error.gyroX.offset;
imu4.wb(:,2) = imu4.wb(:,2) - imu4_error.gyroY.offset;
imu4.wb(:,3) = imu4.wb(:,3) - imu4_error.gyroZ.offset;

imu4.fb(:,1) = imu4.fb(:,1) - imu4_error.accX.offset;
imu4.fb(:,2) = imu4.fb(:,2) - imu4_error.accY.offset;
imu4.fb(:,3) = imu4.fb(:,3) - imu4_error.accZ.offset;



%% GENERATE DATA FOR IMU NUMBER 1
% imu1, imu located at a point different from the center of mass
% imu1MAINcomputed, the data for the center of mass obtained from the IMU1



% relative orientation of the IMU1 with respect to the center of mass of
% the body
imu1.ini_align = [0, 0, 0];
imu1.ini_align_err = deg2rad([0.5, 0.5, 0.5]);


imu1.m_std = [0.005, 0.005, 0.005];

% Assign errors and reference matrices
[imu1] = set_imu(L, imu1, imu1_error);




%% GENERATE DATA FOR IMU NUMBER 2
% imu2, imu located at a point different from the center of mass
% imu2MAINcomputed, the data for the center of mass obtained from the IMU2


% relative orientation of the IMU1 with respect to the center of mass of
% the body
imu2.ini_align = [0, 0, 0];
imu2.ini_align_err = deg2rad([0.5, 0.5, 0.5]);


imu2.m_std = [0.5, 0.5, 0.5];

% Assign errors and reference matrices
[imu2] = set_imu(L, imu2, imu2_error);




%% GENERATE DATA FOR IMU NUMBER 3
% imu3, imu located at a point different from the center of mass
% imu3MAINcomputed, the data for the center of mass obtained from the IMU3

% relative orientation of the IMU3 with respect to the center of mass of
% the body
imu3.ini_align = [0, 0, 0];
imu3.ini_align_err = deg2rad([0.5, 0.5, 0.5]);


imu3.m_std = [0.005, 0.005, 0.005];

% Assign errors and reference matrices
[imu3] = set_imu(L, imu3, imu3_error);



%% GENERATE DATA FOR IMU NUMBER 4
% imu4, imu located at a point different from the center of mass
% imu4MAINcomputed, the data for the center of mass obtained from the IMU4

% relative orientation of the IMU4 with respect to the center of mass of
% the body
imu4.ini_align = [0, 0, 0];
imu4.ini_align_err = deg2rad([0.5, 0.5, 0.5]);


imu4.m_std = [0.005, 0.005, 0.005];

% Assign errors and reference matrices
[imu4] = set_imu(L, imu4, imu4_error);


%% DATA FUSION
[navCM, kf] = arch4_imu2cm_filter_nomag(imu1, imu2, imu3, imu4);

%% PLOT RESULTS
% imu1.wb(:,1) = imu1.wb(:,1) + imu1_error.gyroX.offset;
% imu1.wb(:,2) = imu1.wb(:,2) + imu1_error.gyroY.offset;
% imu1.wb(:,3) = imu1.wb(:,3) + imu1_error.gyroZ.offset;
% 
% imu1.fb(:,1) = imu1.fb(:,1) + imu1_error.accX.offset;
% imu1.fb(:,2) = imu1.fb(:,2) + imu1_error.accY.offset;
% imu1.fb(:,3) = imu1.fb(:,3) + imu1_error.accZ.offset;
% 
% imu2.wb(:,1) = imu2.wb(:,1) + imu2_error.gyroX.offset;
% imu2.wb(:,2) = imu2.wb(:,2) + imu2_error.gyroY.offset;
% imu2.wb(:,3) = imu2.wb(:,3) + imu2_error.gyroZ.offset;
% 
% imu2.fb(:,1) = imu2.fb(:,1) + imu2_error.accX.offset;
% imu2.fb(:,2) = imu2.fb(:,2) + imu2_error.accY.offset;
% imu2.fb(:,3) = imu2.fb(:,3) + imu2_error.accZ.offset;
% 
% imu3.wb(:,1) = imu3.wb(:,1) + imu3_error.gyroX.offset;
% imu3.wb(:,2) = imu3.wb(:,2) + imu3_error.gyroY.offset;
% imu3.wb(:,3) = imu3.wb(:,3) + imu3_error.gyroZ.offset;
% 
% imu3.fb(:,1) = imu3.fb(:,1) + imu3_error.accX.offset;
% imu3.fb(:,2) = imu3.fb(:,2) + imu3_error.accY.offset;
% imu3.fb(:,3) = imu3.fb(:,3) + imu3_error.accZ.offset;
% 
% imu4.wb(:,1) = imu4.wb(:,1) + imu4_error.gyroX.offset;
% imu4.wb(:,2) = imu4.wb(:,2) + imu4_error.gyroY.offset;
% imu4.wb(:,3) = imu4.wb(:,3) + imu4_error.gyroZ.offset;
% 
% imu4.fb(:,1) = imu4.fb(:,1) + imu4_error.accX.offset;
% imu4.fb(:,2) = imu4.fb(:,2) + imu4_error.accY.offset;
% imu4.fb(:,3) = imu4.fb(:,3) + imu4_error.accZ.offset;


[imu1MAIN] = IMU_to_VIMU(imu1);
imu1MAIN.ini_align = deg2rad([0, 0, 0]);
[imu1MAIN.quat, imu1MAIN.euler] = attitude_computer(imu1MAIN);
imu1MAIN.euler = rad2deg(imu1MAIN.euler);

[imu2MAIN] = IMU_to_VIMU(imu2);
imu2MAIN.ini_align = deg2rad([0, 0, 0]);
[imu2MAIN.quat, imu2MAIN.euler] = attitude_computer(imu2MAIN);
imu2MAIN.euler = rad2deg(imu2MAIN.euler);

[imu3MAIN] = IMU_to_VIMU(imu3);
imu3MAIN.ini_align = deg2rad([0, 0, 0]);
[imu3MAIN.quat, imu3MAIN.euler] = attitude_computer(imu3MAIN);
imu3MAIN.euler = rad2deg(imu3MAIN.euler);

[imu4MAIN] = IMU_to_VIMU(imu4);
imu4MAIN.ini_align = deg2rad([0, 0, 0]);
[imu4MAIN.quat, imu4MAIN.euler] = attitude_computer(imu4MAIN);
imu4MAIN.euler = rad2deg(imu4MAIN.euler);


% Plot comparison between true attitude vs EKF output

figure(1)
plot(navCM.t, navCM.qua1(:,1), 'r', navCM.t, navCM.qua1(:,2), 'b', navCM.t, navCM.qua1(:,3), 'g', navCM.t, navCM.qua1(:,4), 'k', ...
    imu1MAIN.t, imu1MAIN.quat(:,1), 'or', imu1MAIN.t, imu1MAIN.quat(:,2), 'ob', imu1MAIN.t, imu1MAIN.quat(:,3), 'og', imu1MAIN.t, imu1MAIN.quat(:,4), 'ok')
xlabel('Time [s]','interpreter','latex')
ylabel('quaternions','interpreter','latex')
legend('q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman','q1', 'q2,', 'q3', 'q4','interpreter','latex')
grid minor
title('Attitude computer vs Kalman filter. Quaternions. IMU 1.','interpreter','latex')
legend('location','southwest')
xlim([imu1.t(1), imu1.t(end)])

figure(2)
plot(navCM.t, navCM.roll1, 'r', navCM.t, navCM.pitch1, 'b', navCM.t, navCM.yaw1, 'g', ...
     imu1MAIN.t, imu1MAIN.euler(:,1), 'or', imu1MAIN.t, imu1MAIN.euler(:,2), 'ob', imu1MAIN.t, imu1MAIN.euler(:,3), 'og')
xlabel('Time [s]','interpreter','latex')
ylabel('Euler angles [deg]','interpreter','latex')
legend('roll Kalman','pitch Kalman','yaw Kalman','roll', 'pitch,', 'yaw','interpreter','latex')
grid minor
title('Euler angles. IMU 1.','interpreter','latex')
legend('location','southwest')
xlim([imu1.t(1), imu1.t(end)])


figure(3)
plot(navCM.t, navCM.qua2(:,1), 'r', navCM.t, navCM.qua2(:,2), 'b', navCM.t, navCM.qua2(:,3), 'g', navCM.t, navCM.qua2(:,4), 'k', ...
     imu2MAIN.t, imu2MAIN.quat(:,1), 'or', imu2MAIN.t, imu2MAIN.quat(:,2), 'ob', imu2MAIN.t, imu2MAIN.quat(:,3), 'og', imu2MAIN.t, imu2MAIN.quat(:,4), 'ok')
xlabel('Time [s]','interpreter','latex')
ylabel('quaternions','interpreter','latex')
legend('q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman','q1', 'q2,', 'q3', 'q4','interpreter','latex')
grid minor
title('Attitude computer vs Kalman filter. Quaternions. IMU 2.','interpreter','latex')
legend('location','southwest')
xlim([imu1.t(1), imu1.t(end)])

figure(4)
plot(navCM.t, navCM.roll2, 'r', navCM.t, navCM.pitch2, 'b', navCM.t, navCM.yaw2, 'g', ...
     imu2MAIN.t, imu2MAIN.euler(:,1), 'or', imu2MAIN.t, imu2MAIN.euler(:,2), 'ob', imu2MAIN.t, imu2MAIN.euler(:,3), 'og')
xlabel('Time [s]','interpreter','latex')
ylabel('Euler angles [deg]','interpreter','latex')
legend('roll Kalman','pitch Kalman','yaw Kalman','roll', 'pitch,', 'yaw','interpreter','latex')
grid minor
title('Euler angles. IMU 2.','interpreter','latex')
legend('location','southwest')
xlim([imu1.t(1), imu1.t(end)])

figure(5)
plot(navCM.t, navCM.qua3(:,1), 'r', navCM.t, navCM.qua3(:,2), 'b', navCM.t, navCM.qua3(:,3), 'g', navCM.t, navCM.qua3(:,4), 'k', ...
    imu3MAIN.t, imu3MAIN.quat(:,1), 'or', imu3MAIN.t, imu3MAIN.quat(:,2), 'ob', imu3MAIN.t, imu3MAIN.quat(:,3), 'og', imu3MAIN.t, imu3MAIN.quat(:,4), 'ok')
xlabel('Time [s]','interpreter','latex')
ylabel('quaternions','interpreter','latex')
legend('q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman','q1', 'q2,', 'q3', 'q4','interpreter','latex')
grid minor
title('Attitude computer vs Kalman filter. Quaternions. IMU 3.','interpreter','latex')
legend('location','southwest')
xlim([imu1.t(1), imu1.t(end)])

figure(6)
plot(navCM.t, navCM.roll3, 'r', navCM.t, navCM.pitch3, 'b', navCM.t, navCM.yaw3, 'g', ...
     imu3MAIN.t, imu3MAIN.euler(:,1), 'or', imu3MAIN.t, imu3MAIN.euler(:,2), 'ob', imu3MAIN.t, imu3MAIN.euler(:,3), 'og')
xlabel('Time [s]','interpreter','latex')
ylabel('Euler angles [deg]','interpreter','latex')
legend('roll Kalman','pitch Kalman','yaw Kalman','roll', 'pitch,', 'yaw','interpreter','latex')
grid minor
title('Euler angles. IMU 3.','interpreter','latex')
legend('location','southwest')
xlim([imu1.t(1), imu1.t(end)])

figure(7)
plot(navCM.t, navCM.qua4(:,1), 'r', navCM.t, navCM.qua4(:,2), 'b', navCM.t, navCM.qua4(:,3), 'g', navCM.t, navCM.qua4(:,4), 'k', ...
    imu4MAIN.t, imu4MAIN.quat(:,1), 'or', imu4MAIN.t, imu4MAIN.quat(:,2), 'ob', imu4MAIN.t, imu4MAIN.quat(:,3), 'og', imu4MAIN.t, imu4MAIN.quat(:,4), 'ok')
xlabel('Time [s]','interpreter','latex')
ylabel('quaternions')
legend('q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman','q1', 'q2,', 'q3', 'q4','interpreter','latex')
grid minor
title('Attitude computer vs Kalman filter. Quaternions. IMU 4.','interpreter','latex')
legend('location','southwest')
xlim([imu1.t(1), imu1.t(end)])

figure(8)
plot(navCM.t, navCM.roll4, 'r', navCM.t, navCM.pitch4, 'b', navCM.t, navCM.yaw4, 'g', ...
     imu4MAIN.t, imu4MAIN.euler(:,1), 'or', imu4MAIN.t, imu4MAIN.euler(:,2), 'ob', imu4MAIN.t, imu4MAIN.euler(:,3), 'og')
xlabel('Time [s]','interpreter','latex')
ylabel('Euler angles [deg]','interpreter','latex')
legend('roll Kalman','pitch Kalman','yaw Kalman','roll', 'pitch,', 'yaw','interpreter','latex')
grid minor
title('Euler angles. IMU 4.','interpreter','latex')
legend('location','southwest')
xlim([imu1.t(1), imu1.t(end)])

%% PLOT RAW DATA


% imu1.wb(:,1) = imu1.wb(:,1) - imu1_error.gyroX.offset;
% imu1.wb(:,2) = imu1.wb(:,2) - imu1_error.gyroY.offset;
% imu1.wb(:,3) = imu1.wb(:,3) - imu1_error.gyroZ.offset;
% 
% imu1.fb(:,1) = imu1.fb(:,1) - imu1_error.accX.offset;
% imu1.fb(:,2) = imu1.fb(:,2) - imu1_error.accY.offset;
% imu1.fb(:,3) = imu1.fb(:,3) - imu1_error.accZ.offset;
% 
% imu2.wb(:,1) = imu2.wb(:,1) - imu2_error.gyroX.offset;
% imu2.wb(:,2) = imu2.wb(:,2) - imu2_error.gyroY.offset;
% imu2.wb(:,3) = imu2.wb(:,3) - imu2_error.gyroZ.offset;
% 
% imu2.fb(:,1) = imu2.fb(:,1) - imu2_error.accX.offset;
% imu2.fb(:,2) = imu2.fb(:,2) - imu2_error.accY.offset;
% imu2.fb(:,3) = imu2.fb(:,3) - imu2_error.accZ.offset;
% 
% imu3.wb(:,1) = imu3.wb(:,1) - imu3_error.gyroX.offset;
% imu3.wb(:,2) = imu3.wb(:,2) - imu3_error.gyroY.offset;
% imu3.wb(:,3) = imu3.wb(:,3) - imu3_error.gyroZ.offset;
% 
% imu3.fb(:,1) = imu3.fb(:,1) - imu3_error.accX.offset;
% imu3.fb(:,2) = imu3.fb(:,2) - imu3_error.accY.offset;
% imu3.fb(:,3) = imu3.fb(:,3) - imu3_error.accZ.offset;
% 
% imu4.wb(:,1) = imu4.wb(:,1) - imu4_error.gyroX.offset;
% imu4.wb(:,2) = imu4.wb(:,2) - imu4_error.gyroY.offset;
% imu4.wb(:,3) = imu4.wb(:,3) - imu4_error.gyroZ.offset;
% 
% imu4.fb(:,1) = imu4.fb(:,1) - imu4_error.accX.offset;
% imu4.fb(:,2) = imu4.fb(:,2) - imu4_error.accY.offset;
% imu4.fb(:,3) = imu4.fb(:,3) - imu4_error.accZ.offset;


figure(9)
plot(imu1.t, imu1.fb(:,1), 'r', imu2.t, imu2.fb(:,1), 'b', imu3.t, imu3.fb(:,1), 'g', imu4.t, imu4.fb(:,1), 'k')
title("Accelerometer X coordinate [$m/s^2$]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Acceleration [$m/s^2$]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best")
grid minor
xlim([imu1.t(1), imu1.t(end)])

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