%% REAL DATA FROM GYROSTABILIZED TABLE
% Author: Laura Train
% Date of the last update June 16 2021
%
% Analyze the data taken from a gyrostabilized table excited by a known
% motion

%% Include paths
matlabrc
clear; clc; close all;

addpath ../../
addpath ../../../synthetic-data/simulation/imu2cm
addpath ../../../synthetic-data/simulation/
addpath ../../../ins/
addpath ../../../conversions/
addpath ../../../kalman/
addpath ../../../data-acquire/data-processing/post-processing/
addpath ../../../data-acquire/data-processing/post-processing/files/
addpath ../../../data-acquire/data-processing/calibration


%% read data

test = csvread('gyrotable1.csv');

[imu1, imu2, imu3, imu4] = read_realdata(test);

load imu1_error.mat
load imu2_error.mat
load imu3_error.mat
load imu4_error.mat

imu1.t = imu1.t - imu1.t(1);
imu2.t = imu2.t - imu2.t(1);
imu3.t = imu3.t - imu3.t(1);
imu4.t = imu4.t - imu4.t(1);

%% choose skew or in-plane configuration

config = 0;
    
imu1.roll0 = 0;
imu1.pitch0 = 180;
imu1.yaw0 = 90;

imu2.roll0 = 0;
imu2.pitch0 = 180;
imu2.yaw0 = 90;

imu3.roll0 = 0;
imu3.pitch0 = 180;
imu3.yaw0 = 90;

imu4.roll0 = 0;
imu4.pitch0 = 180;
imu4.yaw0 = 90;


%% calibration

start_pos = 110480;
imu1_calib.wb = imu1.wb(start_pos:end, :);
imu1_calib.fb = imu1.fb(start_pos:end, :);

imu2_calib.wb = imu2.wb(start_pos:end, :);
imu2_calib.fb = imu2.fb(start_pos:end, :);

imu3_calib.wb = imu3.wb(start_pos:end, :);
imu3_calib.fb = imu3.fb(start_pos:end, :);

imu4_calib.wb = imu4.wb(start_pos:end, :);
imu4_calib.fb = imu4.fb(start_pos:end, :);


[imu1_error.gyroX.offset, imu1_error.gyroX.std] = get_offset(imu1_calib.wb(:,1), 0);
[imu1_error.gyroY.offset, imu1_error.gyroY.std] = get_offset(imu1_calib.wb(:,2), 0);
[imu1_error.gyroZ.offset, imu1_error.gyroZ.std] = get_offset(imu1_calib.wb(:,3), 0);

[imu1_error.accX.offset, imu1_error.accX.std] = get_offset(imu1_calib.fb(:,1), 0);
[imu1_error.accY.offset, imu1_error.accY.std] = get_offset(imu1_calib.fb(:,2), 0);
[imu1_error.accZ.offset, imu1_error.accZ.std] = get_offset(imu1_calib.fb(:,3), -9.81);

[imu2_error.gyroX.offset, imu2_error.gyroX.std] = get_offset(imu2_calib.wb(:,1), 0);
[imu2_error.gyroY.offset, imu2_error.gyroY.std] = get_offset(imu2_calib.wb(:,2), 0);
[imu2_error.gyroZ.offset, imu2_error.gyroZ.std] = get_offset(imu2_calib.wb(:,3), 0);

[imu2_error.accX.offset, imu2_error.accX.std] = get_offset(imu2_calib.fb(:,1), 0);
[imu2_error.accY.offset, imu2_error.accY.std] = get_offset(imu2_calib.fb(:,2), 0);
[imu2_error.accZ.offset, imu2_error.accZ.std] = get_offset(imu2_calib.fb(:,3), -9.81);

[imu3_error.gyroX.offset, imu3_error.gyroX.std] = get_offset(imu3_calib.wb(:,1), 0);
[imu3_error.gyroY.offset, imu3_error.gyroY.std] = get_offset(imu3_calib.wb(:,2), 0);
[imu3_error.gyroZ.offset, imu3_error.gyroZ.std] = get_offset(imu3_calib.wb(:,3), 0);

[imu3_error.accX.offset, imu3_error.accX.std] = get_offset(imu3_calib.fb(:,1), 0);
[imu3_error.accY.offset, imu3_error.accY.std] = get_offset(imu3_calib.fb(:,2), 0);
[imu3_error.accZ.offset, imu3_error.accZ.std] = get_offset(imu3_calib.fb(:,3), -9.81);
    
[imu4_error.gyroX.offset, imu4_error.gyroX.std] = get_offset(imu4_calib.wb(:,1), 0);
[imu4_error.gyroY.offset, imu4_error.gyroY.std] = get_offset(imu4_calib.wb(:,2), 0);
[imu4_error.gyroZ.offset, imu4_error.gyroZ.std] = get_offset(imu4_calib.wb(:,3), 0);

[imu4_error.accX.offset, imu4_error.accX.std] = get_offset(imu4_calib.fb(:,1), 0);
[imu4_error.accY.offset, imu4_error.accY.std] = get_offset(imu4_calib.fb(:,2), 0);
[imu4_error.accZ.offset, imu4_error.accZ.std] = get_offset(imu4_calib.fb(:,3), -9.81);


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


%% INITIALIZE FOR IMU NUMBER 1

% length from the center of mass to IMU
L1 = -0.039;

% relative orientation of the IMU1 with respect to the center of mass of
% the body
imu1.ini_align = [0, 0, 0];
imu1.ini_align_err = deg2rad([0.5, 0.5, 0.5]);

% Assign errors and reference matrices
[imu1] = set_imu(L1, imu1, imu1_error, config);


%% INITIALIZE IMU NUMBER 2

% length from the center of mass to IMU
L2 = -0.013;

% relative orientation of the IMU1 with respect to the center of mass of
% the body
imu2.ini_align = [0, 0, 0];
imu2.ini_align_err = deg2rad([0.5, 0.5, 0.5]);

% Assign errors and reference matrices
[imu2] = set_imu(L2, imu2, imu2_error, config);


%% INITIALIZE IMU NUMBER 3

% length from the center of mass to IMU
L3 = 0.013;

% relative orientation of the IMU3 with respect to the center of mass of
% the body
imu3.ini_align = [0, 0, 0];
imu3.ini_align_err = deg2rad([0.5, 0.5, 0.5]);

% Assign errors and reference matrices
[imu3] = set_imu(L3, imu3, imu3_error, config);


%% INTIALIZE IMU NUMBER 4

% length from the center of mass to IMU
L4 = 0.039;

% relative orientation of the IMU4 with respect to the center of mass of
% the body
imu4.ini_align = [0, 0, 0];
imu4.ini_align_err = deg2rad([0.5, 0.5, 0.5]);

% Assign errors and reference matrices
[imu4] = set_imu(L4, imu4, imu4_error, config);


%% MOVEMENT 1
start_pos_mov1 = 20280;
end_pos_mov1 = 36292;

[mov1] = separate_motion(start_pos_mov1, end_pos_mov1, imu1, imu2, imu3, imu4);
mov1.imu1.t = mov1.imu1.t - mov1.imu1.t(1);
mov1.imu2.t = mov1.imu2.t - mov1.imu2.t(1);
mov1.imu3.t = mov1.imu3.t - mov1.imu3.t(1);
mov1.imu4.t = mov1.imu4.t - mov1.imu4.t(1);

mov1.imu1.ini_align = deg2rad([0, 10, 200]);
mov1.imu2.ini_align = deg2rad([0, 10, 200]);
mov1.imu3.ini_align = deg2rad([0, 10, 200]);
mov1.imu4.ini_align = deg2rad([0, 10, 200]);

% Attitude EKF for each sensor
[mov1.nav1, mov1.kf.nav1] = arch3_imu2cm_filter_nomag(mov1.imu1);
mov1.nav1.yaw = unwrap(mov1.nav1.yaw);
[mov1.nav2, mov1.kf.nav2] = arch3_imu2cm_filter_nomag(mov1.imu2);
mov1.nav2.yaw = unwrap(mov1.nav2.yaw);
[mov1.nav4, mov1.kf.nav4] = arch3_imu2cm_filter_nomag(mov1.imu4);
mov1.nav4.yaw = unwrap(mov1.nav4.yaw);

% Fusion of the four measurements
[mov1.navCM] = attitude_average_arch3(mov1.nav1, mov1.nav2, mov1.nav4, mov1.nav4);

arch3_mov1_navCM = mov1.navCM;
save('arch3_mov1_navCM','arch3_mov1_navCM');

% mov1 = get_sigmas_arch2(mov1);
% simulate real motion
mov1.imuMAIN.t = mov1.imu1.t;
mov1.imuMAIN.ini_align = deg2rad([20, 2.2, 200]);

mov1.roll_true = deg2rad(20)*sin(pi/1.9*mov1.imuMAIN.t);
mov1.pitch_true = deg2rad(10)*cos(pi/1.9*mov1.imuMAIN.t);
mov1.yaw_true = -deg2rad(15)*sin(pi/3.7*mov1.imuMAIN.t);

mov1.wx(1) = 0;
mov1.wy(1) = 0;
mov1.wz(1) = 0;
mov1.dt = diff(mov1.imu1.t);

for i = 2:length(mov1.imu1.t)
    mov1.wx(i) = (mov1.roll_true(i) - mov1.roll_true(i-1))/mov1.dt(i-1);
    mov1.wy(i) = (mov1.pitch_true(i) - mov1.pitch_true(i-1))/mov1.dt(i-1);
    mov1.wz(i) = (mov1.yaw_true(i) - mov1.yaw_true(i-1))/mov1.dt(i-1);
end

% Generate synthetic motion by defining angular velocities
mov1.imuMAIN.wv(:,1) = mov1.wx;
mov1.imuMAIN.wv(:,2) = mov1.wy;
mov1.imuMAIN.wv(:,3) = mov1.wz;

% Generate synthetic motion for the acceleration and magnetic field.
[mov1.imuMAIN] = IMU_simulator(mov1.imuMAIN);
[mov1.imu1_sim] = VIMU_to_IMU(mov1.imuMAIN, mov1.imu1);
[mov1.imu2_sim] = VIMU_to_IMU(mov1.imuMAIN, mov1.imu2);
[mov1.imu3_sim] = VIMU_to_IMU(mov1.imuMAIN, mov1.imu3);
[mov1.imu4_sim] = VIMU_to_IMU(mov1.imuMAIN, mov1.imu4);

for i = 1:length(mov1.imuMAIN.t)
    euler = [mov1.roll_true(i), mov1.pitch_true(i), mov1.yaw_true(i)];
    mov1.quat(i,:) = euler2qua(euler);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(1);

sgtitle('quaternions - KF. Movement 1. Architecture 3.','interpreter','latex')

subplot(4,1,1)
plot(mov1.navCM.t, mov1.navCM.qua(:,1), 'go')
hold on
plot(mov1.navCM.t, mov1.quat(:,1), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q1','interpreter','latex')
legend('q1 KF','q1 th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(4,1,2)
plot(mov1.navCM.t, mov1.navCM.qua(:,2), 'go')
hold on
plot(mov1.navCM.t, mov1.quat(:,2), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q2','interpreter','latex')
legend('q2 KF','q2 th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(4,1,3)
plot(mov1.navCM.t, mov1.navCM.qua(:,3), 'go')
hold on
plot(mov1.navCM.t, mov1.quat(:,3), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q3','interpreter','latex')
legend('q3 KF','q3 th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(4,1,4)
plot(mov1.navCM.t, mov1.navCM.qua(:,4), 'go')
hold on
plot(mov1.navCM.t, mov1.quat(:,4), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q4','interpreter','latex')
legend('q4 KF','q4 th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

% saveas(fh, 'quaternions_motion1.png')

fh = figure(2);
sgtitle('Euler angles - KF. Movement 1. Architecture 3.','interpreter','latex')

subplot(3,1,1)
plot(mov1.navCM.t, mov1.navCM.roll, 'go')
hold on
plot(mov1.navCM.t, rad2deg(mov1.roll_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('roll [deg]','interpreter','latex')
legend('roll KF','roll th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(3,1,2)
plot(mov1.navCM.t, mov1.navCM.pitch, 'go')
hold on
plot(mov1.navCM.t, rad2deg(mov1.pitch_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('pitch [deg]','interpreter','latex')
legend('pitch KF','pitch th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(3,1,3)
plot(mov1.navCM.t, mov1.navCM.yaw, 'go')
hold on
plot(mov1.navCM.t, rad2deg(mov1.yaw_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('yaw [deg]','interpreter','latex')
legend('yaw KF','yaw th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])


%% MOVEMENT 2
start_pos_mov2 = 51795;
end_pos_mov2 = 71444;

[mov2] = separate_motion(start_pos_mov2, end_pos_mov2, imu1, imu2, imu3, imu4);
mov2.imu1.t = mov2.imu1.t - mov2.imu1.t(1);
mov2.imu2.t = mov2.imu2.t - mov2.imu2.t(1);
mov2.imu3.t = mov2.imu3.t - mov2.imu3.t(1);
mov2.imu4.t = mov2.imu4.t - mov2.imu4.t(1);

mov2.imu1.ini_align = deg2rad([0, 0, 0]);
mov2.imu2.ini_align = deg2rad([0, 0, 0]);
mov2.imu3.ini_align = deg2rad([0, 0, 0]);
mov2.imu4.ini_align = deg2rad([0, 0, 0]);

% Attitude EKF for each sensor
[mov2.nav1, mov2.kf.nav1] = arch3_imu2cm_filter_nomag(mov2.imu1);
mov2.nav1.yaw = unwrap(mov2.nav1.yaw);
[mov2.nav2, mov2.kf.nav2] = arch3_imu2cm_filter_nomag(mov2.imu2);
mov2.nav2.yaw = unwrap(mov2.nav2.yaw);
[mov2.nav4, mov2.kf.nav4] = arch3_imu2cm_filter_nomag(mov2.imu4);
mov2.nav4.yaw = unwrap(mov2.nav4.yaw);

% Fusion of the four measurements
[mov2.navCM] = attitude_average_arch3(mov2.nav1, mov2.nav2, mov2.nav4, mov2.nav4);

arch3_mov2_navCM = mov2.navCM;
save('arch3_mov2_navCM','arch3_mov2_navCM');

% mov2 = get_sigmas_arch2(mov2);

% simulate real motion
mov2.imuMAIN.t = mov2.imu1.t;
mov2.imuMAIN.ini_align = deg2rad([-2.2, 20, 0]);

end_first_motion2 = 57254 - start_pos_mov2 ;
% start_second_motion2 = 57564;
end_second_motion2 = 66025 - start_pos_mov2;
end_third_motion2 = end_pos_mov2 - start_pos_mov2 + 1;

mov2.roll_true(1:end_first_motion2) = deg2rad(0.1)*ones(length(mov2.imuMAIN.t(1:end_first_motion2)),1);
mov2.pitch_true(1:end_first_motion2) = deg2rad(0.21)*ones(length(mov2.imuMAIN.t(1:end_first_motion2)),1);
mov2.yaw_true(1:end_first_motion2) = deg2rad(50)*mov2.imuMAIN.t(1:end_first_motion2 );

mov2.roll_true(end_first_motion2:end_second_motion2) = deg2rad(0.48)*ones(length(mov2.imuMAIN.t(end_first_motion2:end_second_motion2)),1);
mov2.pitch_true(end_first_motion2:end_second_motion2) = deg2rad(0.59)*ones(length(mov2.imuMAIN.t(end_first_motion2:end_second_motion2)),1);
mov2.yaw_true(end_first_motion2:end_second_motion2) = deg2rad(100)*(mov2.imuMAIN.t(end_first_motion2:end_second_motion2) - mov2.imuMAIN.t(end_first_motion2)) + mov2.yaw_true(end_first_motion2 - 1);

mov2.roll_true(end_second_motion2:end_third_motion2) = deg2rad(1.09)*ones(length(mov2.imuMAIN.t(end_second_motion2:end_third_motion2)),1);
mov2.pitch_true(end_second_motion2:end_third_motion2) = deg2rad(1.15)*ones(length(mov2.imuMAIN.t(end_second_motion2:end_third_motion2)),1);
mov2.yaw_true(end_second_motion2:end_third_motion2) = deg2rad(150)*(mov2.imuMAIN.t(end_second_motion2:end_third_motion2) - mov2.imuMAIN.t(end_second_motion2)) + mov2.yaw_true(end_second_motion2 - 1);

for i = 1:length(mov2.imuMAIN.t)
    euler = [mov2.roll_true(i), mov2.pitch_true(i), mov2.yaw_true(i)];
    mov2.quat(i,:) = euler2qua(euler);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(3);

sgtitle('quaternions - KF. Movement 2. Architecture 3.','interpreter','latex')

subplot(4,1,1)
plot(mov2.navCM.t, mov2.navCM.qua(:,1), 'go')
hold on
plot(mov2.navCM.t, mov2.quat(:,1), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q1','interpreter','latex')
legend('q1 KF','q1 th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(4,1,2)
plot(mov2.navCM.t, mov2.navCM.qua(:,2), 'go')
hold on
plot(mov2.navCM.t, mov2.quat(:,2), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q2','interpreter','latex')
legend('q2 KF','q2 th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(4,1,3)
plot(mov2.navCM.t, mov2.navCM.qua(:,3), 'go')
hold on
plot(mov2.navCM.t, mov2.quat(:,3), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q3','interpreter','latex')
legend('q3 KF','q3 th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(4,1,4)
plot(mov2.navCM.t, mov2.navCM.qua(:,4), 'go')
hold on
plot(mov2.navCM.t, mov2.quat(:,4), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q4','interpreter','latex')
legend('q4 KF','q4 th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

% saveas(fh, 'quaternions_motion1.png')

fh = figure(4);
sgtitle('Euler angles - KF. Movement 2. Architecture 3.','interpreter','latex')

subplot(3,1,1)
plot(mov2.navCM.t, mov2.navCM.roll, 'go')
hold on
plot(mov2.navCM.t, rad2deg(mov2.roll_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('roll [deg]','interpreter','latex')
legend('roll KF','roll th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(3,1,2)
plot(mov2.navCM.t, mov2.navCM.pitch, 'go')
hold on
plot(mov2.navCM.t, rad2deg(mov2.pitch_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('pitch [deg]','interpreter','latex')
legend('pitch KF','pitch th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(3,1,3)
plot(mov2.navCM.t, mov2.navCM.yaw, 'go')
hold on
plot(mov2.navCM.t, rad2deg(mov2.yaw_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('yaw [deg]','interpreter','latex')
legend('yaw KF','yaw th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])


%% MOVEMENT 3
start_pos_mov3 = 96306;
end_pos_mov3 = 103830;

[mov3] = separate_motion(start_pos_mov3, end_pos_mov3, imu1, imu2, imu3, imu4);

mov3.imu1.t = mov3.imu1.t - mov3.imu1.t(1);
mov3.imu2.t = mov3.imu2.t - mov3.imu2.t(1);
mov3.imu3.t = mov3.imu3.t - mov3.imu3.t(1);
mov3.imu4.t = mov3.imu4.t - mov3.imu4.t(1);

mov3.imu1.ini_align = deg2rad([0.1, 0.1, 20]);
mov3.imu2.ini_align = deg2rad([0.1, 0.1, 20]);
mov3.imu3.ini_align = deg2rad([0.1, 0.1, 20]);
mov3.imu4.ini_align = deg2rad([0.1, 0.1, 20]);

% Attitude EKF for each sensor
[mov3.nav1, mov3.kf.nav1] = arch3_imu2cm_filter_nomag(mov3.imu1);
mov3.nav1.yaw = unwrap(mov3.nav1.yaw);
[mov3.nav2, mov3.kf.nav2] = arch3_imu2cm_filter_nomag(mov3.imu2);
mov3.nav2.yaw = unwrap(mov3.nav2.yaw);
[mov3.nav4, mov3.kf.nav4] = arch3_imu2cm_filter_nomag(mov3.imu4);
mov3.nav4.yaw = unwrap(mov3.nav4.yaw);

% Fusion of the four measurements
[mov3.navCM] = attitude_average_arch3(mov3.nav1, mov3.nav2, mov3.nav4, mov3.nav4);

arch3_mov3_navCM = mov3.navCM;
save('arch3_mov3_navCM','arch3_mov3_navCM');

% mov3 = get_sigmas_arch2(mov3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(5);

sgtitle('quaternions - KF. Movement 3. Architecture 3.','interpreter','latex')

subplot(4,1,1)
plot(mov3.navCM.t, mov3.navCM.qua(:,1), 'go')
hold on
plot(mov3.navCM.t, mov3.quat(:,1), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q1','interpreter','latex')
legend('q1 KF','q1 th','interpreter','latex')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

subplot(4,1,2)
plot(mov3.navCM.t, mov3.navCM.qua(:,2), 'go')
hold on
plot(mov3.navCM.t, mov3.quat(:,2), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q2','interpreter','latex')
legend('q2 KF','q2 th','interpreter','latex')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

subplot(4,1,3)
plot(mov3.navCM.t, mov3.navCM.qua(:,3), 'go')
hold on
plot(mov3.navCM.t, mov3.quat(:,3), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q3','interpreter','latex')
legend('q3 KF','q3 th','interpreter','latex')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

subplot(4,1,4)
plot(mov3.navCM.t, mov3.navCM.qua(:,4), 'go')
hold on
plot(mov3.navCM.t, mov3.quat(:,4), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q4','interpreter','latex')
legend('q4 KF','q4 th','interpreter','latex')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

% saveas(fh, 'quaternions_motion1.png')

fh = figure(6);
sgtitle('Euler angles - KF. Movement 3. Architecture 3.','interpreter','latex')

subplot(3,1,1)
plot(mov3.navCM.t, mov3.navCM.roll, 'go')
hold on
plot(mov3.navCM.t, rad2deg(mov3.roll_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('roll [deg]','interpreter','latex')
legend('roll KF','roll th','interpreter','latex')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])
ylim([-1,1])

subplot(3,1,2)
plot(mov3.navCM.t, mov3.navCM.pitch, 'go')
hold on
plot(mov3.navCM.t, rad2deg(mov3.pitch_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('pitch [deg]','interpreter','latex')
legend('pitch KF','pitch th','interpreter','latex')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])
ylim([-1,1])

subplot(3,1,3)
plot(mov3.navCM.t, mov3.navCM.yaw, 'go')
hold on
plot(mov3.navCM.t, rad2deg(mov3.yaw_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('yaw [deg]','interpreter','latex')
legend('yaw KF','yaw th','interpreter','latex')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])


%% PLOT RAW DATA OF THE WHOLE EXPERIMENT

fh = figure(7);
fh.WindowState = 'maximized';
sgtitle('Accelerometer raw values. [$m/s^2$]','interpreter','latex')

subplot(3,1,1)
plot(imu1.t, imu1.fb(:,1), 'r', imu2.t, imu2.fb(:,1), 'b', imu4.t, imu4.fb(:,1), 'g')
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU4")
legend('location','westoutside')
grid minor
xlim([imu1.t(1), imu1.t(end)])

subplot(3,1,2)
plot(imu1.t, imu1.fb(:,2), 'r', imu2.t, imu2.fb(:,2), 'b', imu4.t, imu4.fb(:,2), 'g')
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU4")
legend('location','westoutside')
grid minor
xlim([imu1.t(1), imu1.t(end)])

subplot(3,1,3)
plot(imu1.t, imu1.fb(:,3), 'r', imu2.t, imu2.fb(:,3), 'b', imu4.t, imu4.fb(:,3), 'g')
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU4")
legend('location','westoutside')
grid minor
xlim([imu1.t(1), imu1.t(end)])

% saveas(fh, 'rawacc.png')

fh = figure(8);
fh.WindowState = 'maximized';
sgtitle('Gyroscope raw values. [$rad/s$].','interpreter','latex')

subplot(3,1,1)
plot(imu1.t, imu1.wb(:,1), 'r', imu2.t, imu2.wb(:,1), 'b', imu4.t, imu4.wb(:,1), 'g')
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU4")
legend('location','westoutside')
grid minor
xlim([imu1.t(1), imu1.t(end)])

subplot(3,1,2)
plot(imu1.t, imu1.wb(:,2), 'r', imu2.t, imu2.wb(:,2), 'b', imu4.t, imu4.wb(:,2), 'g')
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU4")
legend('location','westoutside')
grid minor
xlim([imu1.t(1), imu1.t(end)])

subplot(3,1,3)
plot(imu1.t, imu1.wb(:,3), 'r', imu2.t, imu2.wb(:,3), 'b', imu4.t, imu4.wb(:,3), 'g')
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU4")
legend('location','westoutside')
grid minor
xlim([imu1.t(1), imu1.t(end)])

% saveas(fh, 'rawgyro.png')