% REAL DATA FROM GYROSTABILIZED TABLE
% Author: Laura Train
% Date of the last update June 17 2021
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

test = csvread('gyrotable2.csv');

[imu1, imu2, imu3, imu4] = read_realdata(test);

imu1.t = imu1.t - imu1.t(1);
imu2.t = imu2.t - imu2.t(1);
imu3.t = imu3.t - imu3.t(1);
imu4.t = imu4.t - imu4.t(1);

load imu1_error_multi.mat
load imu2_error_multi.mat
load imu3_error_multi.mat
load imu4_error_multi.mat


%% choose skew or in-plane configuration

config = 1;
    
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


%% calibration

start_pos = 19662;
imu1_calib.wb = imu1.wb(start_pos:end, :);
imu1_calib.fb = imu1.fb(start_pos:end, :);

imu2_calib.wb = imu2.wb(start_pos:end, :);
imu2_calib.fb = imu2.fb(start_pos:end, :);

imu3_calib.wb = imu3.wb(start_pos:end, :);
imu3_calib.fb = imu3.fb(start_pos:end, :);

imu4_calib.wb = imu4.wb(start_pos:end, :);
imu4_calib.fb = imu4.fb(start_pos:end, :);


[imu1_error_multi.gyroX.offset, imu1_error_multi.gyroX.std] = get_offset(imu1_calib.wb(:,1), 0);
[imu1_error_multi.gyroY.offset, imu1_error_multi.gyroY.std] = get_offset(imu1_calib.wb(:,2), 0);
[imu1_error_multi.gyroZ.offset, imu1_error_multi.gyroZ.std] = get_offset(imu1_calib.wb(:,3), 0);

[imu1_error_multi.accX.offset, imu1_error_multi.accX.std] = get_offset(imu1_calib.fb(:,1), 0);
[imu1_error_multi.accY.offset, imu1_error_multi.accY.std] = get_offset(imu1_calib.fb(:,2), 0);
[imu1_error_multi.accZ.offset, imu1_error_multi.accZ.std] = get_offset(imu1_calib.fb(:,3), 9.81);

[imu2_error_multi.gyroX.offset, imu2_error_multi.gyroX.std] = get_offset(imu2_calib.wb(:,1), 0);
[imu2_error_multi.gyroY.offset, imu2_error_multi.gyroY.std] = get_offset(imu2_calib.wb(:,2), 0);
[imu2_error_multi.gyroZ.offset, imu2_error_multi.gyroZ.std] = get_offset(imu2_calib.wb(:,3), 0);

[imu2_error_multi.accX.offset, imu2_error_multi.accX.std] = get_offset(imu2_calib.fb(:,1), -9.81*cosd(30));
[imu2_error_multi.accY.offset, imu2_error_multi.accY.std] = get_offset(imu2_calib.fb(:,2), 0);
[imu2_error_multi.accZ.offset, imu2_error_multi.accZ.std] = get_offset(imu2_calib.fb(:,3), -9.81*sind(30));

[imu3_error_multi.gyroX.offset, imu3_error_multi.gyroX.std] = get_offset(imu3_calib.wb(:,1), 0);
[imu3_error_multi.gyroY.offset, imu3_error_multi.gyroY.std] = get_offset(imu3_calib.wb(:,2), 0);
[imu3_error_multi.gyroZ.offset, imu3_error_multi.gyroZ.std] = get_offset(imu3_calib.wb(:,3), 0);

[imu3_error_multi.accX.offset, imu3_error_multi.accX.std] = get_offset(imu3_calib.fb(:,1), -9.8*cosd(30));
[imu3_error_multi.accY.offset, imu3_error_multi.accY.std] = get_offset(imu3_calib.fb(:,2), 0);
[imu3_error_multi.accZ.offset, imu3_error_multi.accZ.std] = get_offset(imu3_calib.fb(:,3), -9.81*sind(30));
    
[imu4_error_multi.gyroX.offset, imu4_error_multi.gyroX.std] = get_offset(imu4_calib.wb(:,1), 0);
[imu4_error_multi.gyroY.offset, imu4_error_multi.gyroY.std] = get_offset(imu4_calib.wb(:,2), 0);
[imu4_error_multi.gyroZ.offset, imu4_error_multi.gyroZ.std] = get_offset(imu4_calib.wb(:,3), 0);

[imu4_error_multi.accX.offset, imu4_error_multi.accX.std] = get_offset(imu4_calib.fb(:,1), -9.81*cosd(30));
[imu4_error_multi.accY.offset, imu4_error_multi.accY.std] = get_offset(imu4_calib.fb(:,2), 0);
[imu4_error_multi.accZ.offset, imu4_error_multi.accZ.std] = get_offset(imu4_calib.fb(:,3), -9.81*sind(30));


imu1.wb(:,1) = imu1.wb(:,1) - imu1_error_multi.gyroX.offset;
imu1.wb(:,2) = imu1.wb(:,2) - imu1_error_multi.gyroY.offset;
imu1.wb(:,3) = imu1.wb(:,3) - imu1_error_multi.gyroZ.offset;

imu1.fb(:,1) = imu1.fb(:,1) - imu1_error_multi.accX.offset;
imu1.fb(:,2) = imu1.fb(:,2) - imu1_error_multi.accY.offset;
imu1.fb(:,3) = imu1.fb(:,3) - imu1_error_multi.accZ.offset;

imu2.wb(:,1) = imu2.wb(:,1) - imu2_error_multi.gyroX.offset;
imu2.wb(:,2) = imu2.wb(:,2) - imu2_error_multi.gyroY.offset;
imu2.wb(:,3) = imu2.wb(:,3) - imu2_error_multi.gyroZ.offset;

imu2.fb(:,1) = imu2.fb(:,1) - imu2_error_multi.accX.offset;
imu2.fb(:,2) = imu2.fb(:,2) - imu2_error_multi.accY.offset;
imu2.fb(:,3) = imu2.fb(:,3) - imu2_error_multi.accZ.offset;

imu3.wb(:,1) = imu3.wb(:,1) - imu3_error_multi.gyroX.offset;
imu3.wb(:,2) = imu3.wb(:,2) - imu3_error_multi.gyroY.offset;
imu3.wb(:,3) = imu3.wb(:,3) - imu3_error_multi.gyroZ.offset;

imu3.fb(:,1) = imu3.fb(:,1) - imu3_error_multi.accX.offset;
imu3.fb(:,2) = imu3.fb(:,2) - imu3_error_multi.accY.offset;
imu3.fb(:,3) = imu3.fb(:,3) - imu3_error_multi.accZ.offset;

imu4.wb(:,1) = imu4.wb(:,1) - imu4_error_multi.gyroX.offset;
imu4.wb(:,2) = imu4.wb(:,2) - imu4_error_multi.gyroY.offset;
imu4.wb(:,3) = imu4.wb(:,3) - imu4_error_multi.gyroZ.offset;

imu4.fb(:,1) = imu4.fb(:,1) - imu4_error_multi.accX.offset;
imu4.fb(:,2) = imu4.fb(:,2) - imu4_error_multi.accY.offset;
imu4.fb(:,3) = imu4.fb(:,3) - imu4_error_multi.accZ.offset;

%% INITIALIZE IMU NUMBER 1

% length from the center of mass to each IMU
L1 = 0.075;

% relative orientation of the IMU1 with respect to the center of mass of
% the body
imu1.ini_align = [0, 0, 0];
imu1.ini_align_err = deg2rad([2, 2, 2]);

% Assign errors and reference matrices
[imu1] = set_imu(L1, imu1, imu1_error_multi, config);


%% INITIALIZE IMU NUMBER 2

% length from the center of mass to each IMU
L2 = 0.075;

% relative orientation of the IMU1 with respect to the center of mass of
% the body
imu2.ini_align = [0, 0, 0];
imu2.ini_align_err = deg2rad([2, 2, 2]);

% Assign errors and reference matrices
[imu2] = set_imu(L2, imu2, imu2_error_multi, config);


%% INITIALIZE IMU NUMBER 3

% length from the center of mass to each IMU
L3 = 0.075;

% relative orientation of the IMU3 with respect to the center of mass of
% the body
imu3.ini_align = [0, 0, 0];
imu3.ini_align_err = deg2rad([2, 2, 2]);

% Assign errors and reference matrices
[imu3] = set_imu(L3, imu3, imu3_error_multi, config);


%% INITIALIZE IMU NUMBER 4

% length from the center of mass to each IMU
L4 = 0.075;

% relative orientation of the IMU4 with respect to the center of mass of
% the body
imu4.ini_align = [0, 0, 0];
imu4.ini_align_err = deg2rad([2, 2, 2]);

% Assign errors and reference matrices
[imu4] = set_imu(L4, imu4, imu4_error_multi, config);


%% MOVEMENT 1
% start_pos_mov1 = 3448;3186;
start_pos_mov1 = 3549;
end_pos_mov1 = 4647;

[mov1] = separate_motion(start_pos_mov1, end_pos_mov1, imu1, imu2, imu3, imu4);
mov1.imu1.t = mov1.imu1.t - mov1.imu1.t(1);
mov1.imu2.t = mov1.imu2.t - mov1.imu2.t(1);
mov1.imu3.t = mov1.imu3.t - mov1.imu3.t(1);
mov1.imu4.t = mov1.imu4.t - mov1.imu4.t(1);

mov1.imu1.ini_align = deg2rad([20, 2.2, 3]);
mov1.imu2.ini_align = deg2rad([20, 2.2, 3]);
mov1.imu3.ini_align = deg2rad([20, 2.2, 3]);
mov1.imu4.ini_align = deg2rad([20, 2.2, 3]);

[mov1.navCM, mov1.kf] = arch1_imu2cm_filter_nomag(mov1.imu1, mov1.imu2, mov1.imu3, mov1.imu4);

% mov1.navCM.yaw = unwrap(mov1.navCM.yaw);
arch1_mov1_navCM = mov1.navCM;
mov1_time = mov1.imu1.t;

save('mov1_time','mov1_time');
save('arch1_mov1_navCM','arch1_mov1_navCM');


mov1 = get_sigmas(mov1);


% simulate real motion
mov1.imuMAIN.t = mov1.imu1.t;
mov1.imuMAIN.ini_align = deg2rad([20, 2.2, 1.5]);

mov1.roll_true = deg2rad(20)*cos(pi/2.3*mov1.imuMAIN.t);
mov1.pitch_true = deg2rad(2.3)*cos(pi/2.3*mov1.imuMAIN.t);
mov1.yaw_true = deg2rad(0.25)*cos(pi/1.15*mov1.imuMAIN.t) + deg2rad(3);


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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RAW VALUES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(1);

sgtitle('Accelerometer raw values. Motion 1. [$m/s^2$]','interpreter','latex')

subplot(3,1,1)
plot(mov1.imu1.t, mov1.imu1.fb(:,1), 'r', mov1.imu2.t, mov1.imu2.fb(:,1), 'b', mov1.imu3.t, mov1.imu3.fb(:,1), 'g',  mov1.imu4.t, mov1.imu4.fb(:,1), 'k')
hold on
plot(mov1.imu1_sim.t', mov1.imu1_sim.fb(:,1), 'r', mov1.imu2_sim.t', mov1.imu2_sim.fb(:,1), 'b', mov1.imu3_sim.t', mov1.imu3_sim.fb(:,1), 'g',  mov1.imu4_sim.t', mov1.imu4_sim.fb(:,1), 'k','LineWidth',2)
hold off
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(3,1,2)
plot(mov1.imu1.t, mov1.imu1.fb(:,2), 'r', mov1.imu2.t, mov1.imu2.fb(:,2), 'b', mov1.imu3.t, mov1.imu3.fb(:,2), 'g',  mov1.imu4.t, mov1.imu4.fb(:,2), 'k')
hold on
plot(mov1.imu1_sim.t', mov1.imu1_sim.fb(:,2), 'r', mov1.imu2_sim.t', mov1.imu2_sim.fb(:,2), 'b', mov1.imu3_sim.t', mov1.imu3_sim.fb(:,2), 'g',  mov1.imu4_sim.t', mov1.imu4_sim.fb(:,2), 'k','LineWidth',2)
hold off
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(3,1,3)
plot(mov1.imu1.t, mov1.imu1.fb(:,3), 'r', mov1.imu2.t, mov1.imu2.fb(:,3), 'b', mov1.imu3.t, mov1.imu3.fb(:,3), 'g',  mov1.imu4.t, mov1.imu4.fb(:,3), 'k')
hold on
plot(mov1.imu1_sim.t', mov1.imu1_sim.fb(:,3), 'r', mov1.imu2_sim.t', mov1.imu2_sim.fb(:,3), 'b', mov1.imu3_sim.t', mov1.imu3_sim.fb(:,3), 'g',  mov1.imu4_sim.t', mov1.imu4_sim.fb(:,3), 'k','LineWidth',2)
hold off
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])
% saveas(fh, 'rawacc_motion1.png')


fh = figure(2);
fh.WindowState = 'maximized';

sgtitle('Gyroscope raw values. Motion 1. [$deg/s$].','interpreter','latex')

subplot(3,1,1)
plot(mov1.imu1.t, rad2deg(mov1.imu1.wb(:,1)), 'r', mov1.imu2.t, rad2deg(mov1.imu2.wb(:,1)), 'b', mov1.imu3.t, rad2deg(mov1.imu3.wb(:,1)), 'g',  mov1.imu4.t, rad2deg(mov1.imu4.wb(:,1)), 'k')
hold on
plot(mov1.imu1_sim.t', rad2deg(mov1.imu1_sim.wb(:,1)), 'r', mov1.imu2_sim.t', rad2deg(mov1.imu2_sim.wb(:,1)), 'b', mov1.imu3_sim.t', rad2deg(mov1.imu3_sim.wb(:,1)), 'g',  mov1.imu4_sim.t', rad2deg(mov1.imu4_sim.wb(:,1)), 'k','LineWidth',2)
hold off
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(3,1,2)
plot(mov1.imu1.t, rad2deg(mov1.imu1.wb(:,2)), 'r', mov1.imu2.t, rad2deg(mov1.imu2.wb(:,2)), 'b', mov1.imu3.t, rad2deg(mov1.imu3.wb(:,2)), 'g',  mov1.imu4.t, rad2deg(mov1.imu4.wb(:,2)), 'k')
hold on
plot(mov1.imu1_sim.t', rad2deg(mov1.imu1_sim.wb(:,2)), 'r', mov1.imu2_sim.t', rad2deg(mov1.imu2_sim.wb(:,2)), 'b', mov1.imu3_sim.t', rad2deg(mov1.imu3_sim.wb(:,2)), 'g',  mov1.imu4_sim.t', rad2deg(mov1.imu4_sim.wb(:,2)), 'k','LineWidth',2)
hold off
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(3,1,3)
plot(mov1.imu1.t, rad2deg(mov1.imu1.wb(:,3)), 'r', mov1.imu2.t, rad2deg(mov1.imu2.wb(:,3)), 'b', mov1.imu3.t, rad2deg(mov1.imu3.wb(:,3)), 'g',  mov1.imu4.t, rad2deg(mov1.imu4.wb(:,3)), 'k')
hold on
plot(mov1.imu1_sim.t', rad2deg(mov1.imu1_sim.wb(:,3)), 'r', mov1.imu2_sim.t', rad2deg(mov1.imu2_sim.wb(:,3)), 'b', mov1.imu3_sim.t', rad2deg(mov1.imu3_sim.wb(:,3)), 'g',  mov1.imu4_sim.t', rad2deg(mov1.imu4_sim.wb(:,3)), 'k','LineWidth',2)
hold off
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])


% saveas(fh, 'rawgyro_motion1.png')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(3);

sgtitle('quaternions - KF. Movement 1. Architecture 1.','interpreter','latex')

subplot(4,1,1)
plot(mov1.navCM.t, mov1.navCM.qua(:,1), 'ro')
hold on
plot(mov1.navCM.t, mov1.quat(:,1), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q1','interpreter','latex')
legend('q1 KF','q1 th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(4,1,2)
plot(mov1.navCM.t, mov1.navCM.qua(:,2), 'ro')
hold on
plot(mov1.navCM.t, mov1.quat(:,2), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q2','interpreter','latex')
legend('q2 KF','q2 th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(4,1,3)
plot(mov1.navCM.t, mov1.navCM.qua(:,3), 'ro')
hold on
plot(mov1.navCM.t, mov1.quat(:,3), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q3','interpreter','latex')
legend('q3 KF','q3 th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(4,1,4)
plot(mov1.navCM.t, mov1.navCM.qua(:,4), 'ro')
hold on
plot(mov1.navCM.t, mov1.quat(:,4), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q4','interpreter','latex')
legend('q4 KF','q4 th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

% saveas(fh, 'quaternions_motion1.png')

fh = figure(4);
sgtitle('Euler angles - KF. Movement 1. Architecture 1.','interpreter','latex')

subplot(3,1,1)
plot(mov1.navCM.t, mov1.navCM.roll, 'ro')
hold on
plot(mov1.navCM.t, rad2deg(mov1.roll_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('roll [deg]','interpreter','latex')
legend('roll KF','roll th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(3,1,2)
plot(mov1.navCM.t, mov1.navCM.pitch, 'ro')
hold on
plot(mov1.navCM.t, rad2deg(mov1.pitch_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('pitch [deg]','interpreter','latex')
legend('pitch KF','pitch th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])

subplot(3,1,3)
plot(mov1.navCM.t, mov1.navCM.yaw, 'ro')
hold on
plot(mov1.navCM.t, rad2deg(mov1.yaw_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('yaw [deg]','interpreter','latex')
legend('yaw KF','yaw th','interpreter','latex')
grid minor
xlim([mov1.imu1.t(1), mov1.imu1.t(end)])


% saveas(fh, 'eulerangles_motion1.png')


%% MOVEMENT 2

start_pos_mov2 = 5413;
end_pos_mov2 = 6799;

[mov2] = separate_motion(start_pos_mov2, end_pos_mov2, imu1, imu2, imu3, imu4);
mov2.imu1.t = mov2.imu1.t - mov2.imu1.t(1);
mov2.imu2.t = mov2.imu2.t - mov2.imu2.t(1);
mov2.imu3.t = mov2.imu3.t - mov2.imu3.t(1);
mov2.imu4.t = mov2.imu4.t - mov2.imu4.t(1);

mov2.imu1.ini_align = deg2rad([-2.2, 20, 5]);
mov2.imu2.ini_align = deg2rad([-2.2, 20, 5]);
mov2.imu3.ini_align = deg2rad([-2.2, 20, 5]);
mov2.imu4.ini_align = deg2rad([-2.2, 20, 5]);

[mov2.navCM, mov2.kf] = arch1_imu2cm_filter_nomag(mov2.imu1, mov2.imu2, mov2.imu3, mov2.imu4);

arch1_mov2_navCM = mov2.navCM;
mov2_time = mov2.imu1.t;

save('mov2_time','mov2_time');
save('arch1_mov2_navCM','arch1_mov2_navCM');


% mov2 = get_sigmas(mov2);


% simulate real motion
mov2.imuMAIN.t = mov2.imu1.t;
mov2.imuMAIN.ini_align = deg2rad([-2.2, 20, 0]);

mov2.roll_true = -deg2rad(2.3)*cos(pi/2.3*mov2.imuMAIN.t);
mov2.pitch_true = deg2rad(20)*cos(pi/2.3*mov2.imuMAIN.t);
mov2.yaw_true = -deg2rad(0.5)*cos(pi/1.15*mov2.imuMAIN.t) + deg2rad(5);

mov2.wx(1) = 0;
mov2.wy(1) = 0;
mov2.wz(1) = 0;
mov2.dt = diff(mov2.imu1.t);

for i = 2:length(mov2.imu1.t)
    mov2.wx(i) = (mov2.roll_true(i) - mov2.roll_true(i-1))/mov2.dt(i-1);
    mov2.wy(i) = (mov2.pitch_true(i) - mov2.pitch_true(i-1))/mov2.dt(i-1);
    mov2.wz(i) = (mov2.yaw_true(i) - mov2.yaw_true(i-1))/mov2.dt(i-1);
end
% Generate synthetic motion by defining angular velocities
mov2.imuMAIN.wv(:,1) = mov2.wx;
mov2.imuMAIN.wv(:,2) = mov2.wy;
mov2.imuMAIN.wv(:,3) = mov2.wz;

% Generate synthetic motion for the acceleration and magnetic field.
[mov2.imuMAIN] = IMU_simulator(mov2.imuMAIN);
[mov2.imu1_sim] = VIMU_to_IMU(mov2.imuMAIN, mov2.imu1);
[mov2.imu2_sim] = VIMU_to_IMU(mov2.imuMAIN, mov2.imu2);
[mov2.imu3_sim] = VIMU_to_IMU(mov2.imuMAIN, mov2.imu3);
[mov2.imu4_sim] = VIMU_to_IMU(mov2.imuMAIN, mov2.imu4);

for i = 1:length(mov2.imuMAIN.t)
    euler = [mov2.roll_true(i), mov2.pitch_true(i), mov2.yaw_true(i)];
    mov2.quat(i,:) = euler2qua(euler);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RAW VALUES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(5);

sgtitle('Accelerometer raw values. Motion 2. [$m/s^2$]','interpreter','latex')

subplot(3,1,1)
plot(mov2.imu1.t, mov2.imu1.fb(:,1), 'r', mov2.imu2.t, mov2.imu2.fb(:,1), 'b', mov2.imu3.t, mov2.imu3.fb(:,1), 'g',  mov2.imu4.t, mov2.imu4.fb(:,1), 'k')
hold on
plot(mov2.imu1_sim.t', mov2.imu1_sim.fb(:,1), 'r', mov2.imu2_sim.t', mov2.imu2_sim.fb(:,1), 'b', mov2.imu3_sim.t', mov2.imu3_sim.fb(:,1), 'g',  mov2.imu4_sim.t', mov2.imu4_sim.fb(:,1), 'k','LineWidth',2)
hold off
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(3,1,2)
plot(mov2.imu1.t, mov2.imu1.fb(:,2), 'r', mov2.imu2.t, mov2.imu2.fb(:,2), 'b', mov2.imu3.t, mov2.imu3.fb(:,2), 'g',  mov2.imu4.t, mov2.imu4.fb(:,2), 'k')
hold on
plot(mov2.imu1_sim.t', mov2.imu1_sim.fb(:,2), 'r', mov2.imu2_sim.t', mov2.imu2_sim.fb(:,2), 'b', mov2.imu3_sim.t', mov2.imu3_sim.fb(:,2), 'g',  mov2.imu4_sim.t', mov2.imu4_sim.fb(:,2), 'k','LineWidth',2)
hold off
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(3,1,3)
plot(mov2.imu1.t, mov2.imu1.fb(:,3), 'r', mov2.imu2.t, mov2.imu2.fb(:,3), 'b', mov2.imu3.t, mov2.imu3.fb(:,3), 'g',  mov2.imu4.t, mov2.imu4.fb(:,3), 'k')
hold on
plot(mov2.imu1_sim.t', mov2.imu1_sim.fb(:,3), 'r', mov2.imu2_sim.t', mov2.imu2_sim.fb(:,3), 'b', mov2.imu3_sim.t', mov2.imu3_sim.fb(:,3), 'g',  mov2.imu4_sim.t', mov2.imu4_sim.fb(:,3), 'k','LineWidth',2)
hold off
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])
% saveas(fh, 'rawacc_motion2.png')


fh = figure(6);

sgtitle('Gyroscope raw values. Motion 2. [$deg/s$].','interpreter','latex')

subplot(3,1,1)
plot(mov2.imu1.t, rad2deg(mov2.imu1.wb(:,1)), 'r', mov2.imu2.t, rad2deg(mov2.imu2.wb(:,1)), 'b', mov2.imu3.t, rad2deg(mov2.imu3.wb(:,1)), 'g',  mov2.imu4.t, rad2deg(mov2.imu4.wb(:,1)), 'k')
hold on
plot(mov2.imu1_sim.t', rad2deg(mov2.imu1_sim.wb(:,1)), 'r', mov2.imu2_sim.t', rad2deg(mov2.imu2_sim.wb(:,1)), 'b', mov2.imu3_sim.t', rad2deg(mov2.imu3_sim.wb(:,1)), 'g',  mov2.imu4_sim.t', rad2deg(mov2.imu4_sim.wb(:,1)), 'k','LineWidth',2)
hold off
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(3,1,2)
plot(mov2.imu1.t, rad2deg(mov2.imu1.wb(:,2)), 'r', mov2.imu2.t, rad2deg(mov2.imu2.wb(:,2)), 'b', mov2.imu3.t, rad2deg(mov2.imu3.wb(:,2)), 'g',  mov2.imu4.t, rad2deg(mov2.imu4.wb(:,2)), 'k')
hold on
plot(mov2.imu1_sim.t', rad2deg(mov2.imu1_sim.wb(:,2)), 'r', mov2.imu2_sim.t', rad2deg(mov2.imu2_sim.wb(:,2)), 'b', mov2.imu3_sim.t', rad2deg(mov2.imu3_sim.wb(:,2)), 'g',  mov2.imu4_sim.t', rad2deg(mov2.imu4_sim.wb(:,2)), 'k','LineWidth',2)
hold off
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(3,1,3)
plot(mov2.imu1.t, rad2deg(mov2.imu1.wb(:,3)), 'r', mov2.imu2.t, rad2deg(mov2.imu2.wb(:,3)), 'b', mov2.imu3.t, rad2deg(mov2.imu3.wb(:,3)), 'g',  mov2.imu4.t, rad2deg(mov2.imu4.wb(:,3)), 'k')
hold on
plot(mov2.imu1_sim.t', rad2deg(mov2.imu1_sim.wb(:,3)), 'r', mov2.imu2_sim.t', rad2deg(mov2.imu2_sim.wb(:,3)), 'b', mov2.imu3_sim.t', rad2deg(mov2.imu3_sim.wb(:,3)), 'g',  mov2.imu4_sim.t', rad2deg(mov2.imu4_sim.wb(:,3)), 'k','LineWidth',2)
hold off
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])


% saveas(fh, 'rawgyro_motion2.png')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(7);

sgtitle('quaternions - KF. Movement 2. Architecture 1.','interpreter','latex')

subplot(4,1,1)
plot(mov2.navCM.t, mov2.navCM.qua(:,1), 'ro')
hold on
plot(mov2.navCM.t, mov2.quat(:,1), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q1','interpreter','latex')
legend('q1 KF','q1 th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(4,1,2)
plot(mov2.navCM.t, mov2.navCM.qua(:,2), 'ro')
hold on
plot(mov2.navCM.t, mov2.quat(:,2), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q2','interpreter','latex')
legend('q2 KF','q2 th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(4,1,3)
plot(mov2.navCM.t, mov2.navCM.qua(:,3), 'ro')
hold on
plot(mov2.navCM.t, mov2.quat(:,3), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q3','interpreter','latex')
legend('q3 KF','q3 th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(4,1,4)
plot(mov2.navCM.t, mov2.navCM.qua(:,4), 'ro')
hold on
plot(mov2.navCM.t, mov2.quat(:,4), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q4','interpreter','latex')
legend('q4 KF','q4 th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

% saveas(fh, 'quaternions_motion1.png')

fh = figure(8);
sgtitle('Euler angles - KF. Movement 2. Architecture 1.','interpreter','latex')

subplot(3,1,1)
plot(mov2.navCM.t, mov2.navCM.roll, 'ro')
hold on
plot(mov2.navCM.t, rad2deg(mov2.roll_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('roll [deg]','interpreter','latex')
legend('roll KF','roll th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(3,1,2)
plot(mov2.navCM.t, mov2.navCM.pitch, 'ro')
hold on
plot(mov2.navCM.t, rad2deg(mov2.pitch_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('pitch [deg]','interpreter','latex')
legend('pitch KF','pitch th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])

subplot(3,1,3)
plot(mov2.navCM.t, mov2.navCM.yaw, 'ro')
hold on
plot(mov2.navCM.t, rad2deg(mov2.yaw_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('yaw [deg]','interpreter','latex')
legend('yaw KF','yaw th','interpreter','latex')
grid minor
xlim([mov2.imu1.t(1), mov2.imu1.t(end)])


% saveas(fh, 'eulerangles_motion1.png')

%% MOVEMENT 3
start_pos_mov3 = 7615;
end_pos_mov3 = 9111;

[mov3] = separate_motion(start_pos_mov3, end_pos_mov3, imu1, imu2, imu3, imu4);
mov3.imu1.t = mov3.imu1.t - mov3.imu1.t(1);
mov3.imu2.t = mov3.imu2.t - mov3.imu2.t(1);
mov3.imu3.t = mov3.imu3.t - mov3.imu3.t(1);
mov3.imu4.t = mov3.imu4.t - mov3.imu4.t(1);

mov3.imu1.ini_align = deg2rad([0.1, 0.1, 20]);
mov3.imu2.ini_align = deg2rad([0.1, 0.1, 20]);
mov3.imu3.ini_align = deg2rad([0.1, 0.1, 20]);
mov3.imu4.ini_align = deg2rad([0.1, 0.1, 20]);

[mov3.navCM, mov3.kf] = arch1_imu2cm_filter_nomag(mov3.imu1, mov3.imu2, mov3.imu3, mov3.imu4);

arch1_mov3_navCM = mov3.navCM;
mov3_time = mov3.imu1.t;

save('mov3_time','mov3_time');
save('arch1_mov3_navCM','arch1_mov3_navCM');

% mov3 = get_sigmas(mov3);

% simulate real motion
mov3.imuMAIN.t = mov3.imu1.t;

mov3.roll_true = deg2rad(0.1)*cos(pi/2.3*mov3.imuMAIN.t);
% mov3.pitch_true = deg2rad(0.1)*cos(pi/2.3*mov3.imuMAIN.t);
mov3.pitch_true = deg2rad(0.1)*cos(pi/1.15*mov3.imuMAIN.t);
mov3.yaw_true = deg2rad(20)*cos(pi/2.3*mov3.imuMAIN.t);

mov3.wx(1) = 0;
mov3.wy(1) = 0;
mov3.wz(1) = 0;
mov3.dt = diff(mov3.imu1.t);
for i = 2:length(mov3.imu1.t)
    mov3.wx(i) = (mov3.roll_true(i) - mov3.roll_true(i-1))/mov3.dt(i-1);
    mov3.wy(i) = (mov3.pitch_true(i) - mov3.pitch_true(i-1))/mov3.dt(i-1);
    mov3.wz(i) = (mov3.yaw_true(i) - mov3.yaw_true(i-1))/mov3.dt(i-1);
end
% Generate synthetic motion by defining angular velocities
% wx = -deg2rad(24)*sin(pi/2.3*mov3.imuMAIN.t);
% wy = -deg2rad(2.2)*sin(pi/2.3*mov3.imuMAIN.t);
% wz = -deg2rad(1)*sin(pi/2.3*mov3.imuMAIN.t);
mov3.imuMAIN.wv(:,1) = mov3.wx;
mov3.imuMAIN.wv(:,2) = mov3.wy;
mov3.imuMAIN.wv(:,3) = mov3.wz;
mov3.imuMAIN.ini_align = deg2rad([0, 0.02, 20]);

% Generate synthetic motion for the acceleration and magnetic field.
[mov3.imuMAIN] = IMU_simulator(mov3.imuMAIN);
[mov3.imu1_sim] = VIMU_to_IMU(mov3.imuMAIN, mov3.imu1);
[mov3.imu2_sim] = VIMU_to_IMU(mov3.imuMAIN, mov3.imu2);
[mov3.imu3_sim] = VIMU_to_IMU(mov3.imuMAIN, mov3.imu3);
[mov3.imu4_sim] = VIMU_to_IMU(mov3.imuMAIN, mov3.imu4);

for i = 1:length(mov3.imuMAIN.t)
    euler = [mov3.roll_true(i), mov3.pitch_true(i), mov3.yaw_true(i)];
    mov3.quat(i,:) = euler2qua(euler);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RAW VALUES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(9);

sgtitle('Accelerometer raw values. Motion 3. [$m/s^2$]','interpreter','latex')

subplot(3,1,1)
plot(mov3.imu1.t, mov3.imu1.fb(:,1), 'r', mov3.imu2.t, mov3.imu2.fb(:,1), 'b', mov3.imu3.t, mov3.imu3.fb(:,1), 'g',  mov3.imu4.t, mov3.imu4.fb(:,1), 'k')
hold on
plot(mov3.imu1_sim.t', mov3.imu1_sim.fb(:,1), 'r', mov3.imu2_sim.t', mov3.imu2_sim.fb(:,1), 'b', mov3.imu3_sim.t', mov3.imu3_sim.fb(:,1), 'g',  mov3.imu4_sim.t', mov3.imu4_sim.fb(:,1), 'k','LineWidth',2)
hold off
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

subplot(3,1,2)
plot(mov3.imu1.t, mov3.imu1.fb(:,2), 'r', mov3.imu2.t, mov3.imu2.fb(:,2), 'b', mov3.imu3.t, mov3.imu3.fb(:,2), 'g',  mov3.imu4.t, mov3.imu4.fb(:,2), 'k')
hold on
plot(mov3.imu1_sim.t', mov3.imu1_sim.fb(:,2), 'r', mov3.imu2_sim.t', mov3.imu2_sim.fb(:,2), 'b', mov3.imu3_sim.t', mov3.imu3_sim.fb(:,2), 'g',  mov3.imu4_sim.t', mov3.imu4_sim.fb(:,2), 'k','LineWidth',2)
hold off
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

subplot(3,1,3)
plot(mov3.imu1.t, mov3.imu1.fb(:,3), 'r', mov3.imu2.t, mov3.imu2.fb(:,3), 'b', mov3.imu3.t, mov3.imu3.fb(:,3), 'g',  mov3.imu4.t, mov3.imu4.fb(:,3), 'k')
hold on
plot(mov3.imu1_sim.t', mov3.imu1_sim.fb(:,3), 'r', mov3.imu2_sim.t', mov3.imu2_sim.fb(:,3), 'b', mov3.imu3_sim.t', mov3.imu3_sim.fb(:,3), 'g',  mov3.imu4_sim.t', mov3.imu4_sim.fb(:,3), 'k','LineWidth',2)
hold off
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])
% saveas(fh, 'rawacc_motion2.png')


fh = figure(10);

sgtitle('Gyroscope raw values. Motion 3. [$deg/s$].','interpreter','latex')

subplot(3,1,1)
plot(mov3.imu1.t, rad2deg(mov3.imu1.wb(:,1)), 'r', mov3.imu2.t, rad2deg(mov3.imu2.wb(:,1)), 'b', mov3.imu3.t, rad2deg(mov3.imu3.wb(:,1)), 'g',  mov3.imu4.t, rad2deg(mov3.imu4.wb(:,1)), 'k')
hold on
plot(mov3.imu1_sim.t', rad2deg(mov3.imu1_sim.wb(:,1)), 'r', mov3.imu2_sim.t', rad2deg(mov3.imu2_sim.wb(:,1)), 'b', mov3.imu3_sim.t', rad2deg(mov3.imu3_sim.wb(:,1)), 'g',  mov3.imu4_sim.t', rad2deg(mov3.imu4_sim.wb(:,1)), 'k','LineWidth',2)
hold off
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

subplot(3,1,2)
plot(mov3.imu1.t, rad2deg(mov3.imu1.wb(:,2)), 'r', mov3.imu2.t, rad2deg(mov3.imu2.wb(:,2)), 'b', mov3.imu3.t, rad2deg(mov3.imu3.wb(:,2)), 'g',  mov3.imu4.t, rad2deg(mov3.imu4.wb(:,2)), 'k')
hold on
plot(mov3.imu1_sim.t', rad2deg(mov3.imu1_sim.wb(:,2)), 'r', mov3.imu2_sim.t', rad2deg(mov3.imu2_sim.wb(:,2)), 'b', mov3.imu3_sim.t', rad2deg(mov3.imu3_sim.wb(:,2)), 'g',  mov3.imu4_sim.t', rad2deg(mov3.imu4_sim.wb(:,2)), 'k','LineWidth',2)
hold off
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

subplot(3,1,3)
plot(mov3.imu1.t, rad2deg(mov3.imu1.wb(:,3)), 'r', mov3.imu2.t, rad2deg(mov3.imu2.wb(:,3)), 'b', mov3.imu3.t, rad2deg(mov3.imu3.wb(:,3)), 'g',  mov3.imu4.t, rad2deg(mov3.imu4.wb(:,3)), 'k')
hold on
plot(mov3.imu1_sim.t', rad2deg(mov3.imu1_sim.wb(:,3)), 'r', mov3.imu2_sim.t', rad2deg(mov3.imu2_sim.wb(:,3)), 'b', mov3.imu3_sim.t', rad2deg(mov3.imu3_sim.wb(:,3)), 'g',  mov3.imu4_sim.t', rad2deg(mov3.imu4_sim.wb(:,3)), 'k','LineWidth',2)
hold off
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])


% saveas(fh, 'rawgyro_motion2.png')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(11);

sgtitle('quaternions - KF. Movement 3. Architecture 1.','interpreter','latex')

subplot(4,1,1)
plot(mov3.navCM.t, mov3.navCM.qua(:,1), 'ro')
hold on
plot(mov3.navCM.t, mov3.quat(:,1), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q1','interpreter','latex')
legend('q1 KF','q1 th','interpreter','latex')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

subplot(4,1,2)
plot(mov3.navCM.t, mov3.navCM.qua(:,2), 'ro')
hold on
plot(mov3.navCM.t, mov3.quat(:,2), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q2','interpreter','latex')
legend('q2 KF','q2 th','interpreter','latex')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

subplot(4,1,3)
plot(mov3.navCM.t, mov3.navCM.qua(:,3), 'ro')
hold on
plot(mov3.navCM.t, mov3.quat(:,3), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q3','interpreter','latex')
legend('q3 KF','q3 th','interpreter','latex')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

subplot(4,1,4)
plot(mov3.navCM.t, mov3.navCM.qua(:,4), 'ro')
hold on
plot(mov3.navCM.t, mov3.quat(:,4), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q4','interpreter','latex')
legend('q4 KF','q4 th','interpreter','latex')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

% saveas(fh, 'quaternions_motion1.png')

fh = figure(12);
sgtitle('Euler angles - KF. Movement 3. Architecture 1.','interpreter','latex')

subplot(3,1,1)
plot(mov3.navCM.t, mov3.navCM.roll, 'ro')
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
plot(mov3.navCM.t, mov3.navCM.pitch, 'ro')
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
plot(mov3.navCM.t, mov3.navCM.yaw, 'ro')
hold on
plot(mov3.navCM.t, rad2deg(mov3.yaw_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('yaw [deg]','interpreter','latex')
legend('yaw KF','yaw th','interpreter','latex')
grid minor
xlim([mov3.imu1.t(1), mov3.imu1.t(end)])

% saveas(fh, 'eulerangles_motion1.png')
 

%% MOVEMENT 4
start_pos_mov4 = 9830;
end_pos_mov4 = 11207;

[mov4] = separate_motion(start_pos_mov4, end_pos_mov4, imu1, imu2, imu3, imu4);
mov4.imu1.t = mov4.imu1.t - mov4.imu1.t(1);
mov4.imu2.t = mov4.imu2.t - mov4.imu2.t(1);
mov4.imu3.t = mov4.imu3.t - mov4.imu3.t(1);
mov4.imu4.t = mov4.imu4.t - mov4.imu4.t(1);

mov4.imu1.ini_align = deg2rad([20, 5.2, 10]);
mov4.imu2.ini_align = deg2rad([20, 5.2, 10]);
mov4.imu3.ini_align = deg2rad([20, 5.2, 10]);
mov4.imu4.ini_align = deg2rad([20, 5.2, 10]);

[mov4.navCM, mov4.kf] = arch1_imu2cm_filter_nomag(mov4.imu1, mov4.imu2, mov4.imu3, mov4.imu4);

% mov4.navCM.yaw = unwrap(mov4.navCM.yaw);
arch1_mov4_navCM = mov4.navCM;
mov4_time = mov4.imu1.t;

save('mov4_time','mov4_time');
save('arch1_mov4_navCM','arch1_mov4_navCM');


% mov4 = get_sigmas(mov4);


% simulate real motion
mov4.imuMAIN.t = mov4.imu1.t;

mov4.roll_true = deg2rad(20)*cos(pi/2.3*mov4.imuMAIN.t )  ;
mov4.pitch_true = deg2rad(6)*cos(pi/2.3*mov4.imuMAIN.t - 2.3/2);
mov4.yaw_true = deg2rad(5)*cos(pi/2.3*mov4.imuMAIN.t) + deg2rad(15);


mov4.wx(1) = 0;
mov4.wy(1) = 0;
mov4.wz(1) = 0;
mov4.dt = diff(mov4.imu1.t);

for i = 2:length(mov4.imu1.t)
    mov4.wx(i) = (mov4.roll_true(i) - mov4.roll_true(i-1))/mov4.dt(i-1);
    mov4.wy(i) = (mov4.pitch_true(i) - mov4.pitch_true(i-1))/mov4.dt(i-1);
    mov4.wz(i) = (mov4.yaw_true(i) - mov4.yaw_true(i-1))/mov4.dt(i-1);
end
% Generate synthetic motion by defining angular velocities

mov4.imuMAIN.wv(:,1) = mov4.wx;
mov4.imuMAIN.wv(:,2) = mov4.wy;
mov4.imuMAIN.wv(:,3) = mov4.wz;
mov4.imuMAIN.ini_align = deg2rad([20, 5.2, 5]);

% Generate synthetic motion for the acceleration and magnetic field.
[mov4.imuMAIN] = IMU_simulator(mov4.imuMAIN);
[mov4.imu1_sim] = VIMU_to_IMU(mov4.imuMAIN, mov4.imu1);
[mov4.imu2_sim] = VIMU_to_IMU(mov4.imuMAIN, mov4.imu2);
[mov4.imu3_sim] = VIMU_to_IMU(mov4.imuMAIN, mov4.imu3);
[mov4.imu4_sim] = VIMU_to_IMU(mov4.imuMAIN, mov4.imu4);

for i = 1:length(mov4.imuMAIN.t)
    euler = [mov4.roll_true(i), mov4.pitch_true(i), mov4.yaw_true(i)];
    mov4.quat(i,:) = euler2qua(euler);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RAW VALUES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(13);

sgtitle('Accelerometer raw values. Motion 4. [$m/s^2$]','interpreter','latex')

subplot(3,1,1)
plot(mov4.imu1.t, mov4.imu1.fb(:,1), 'r', mov4.imu2.t, mov4.imu2.fb(:,1), 'b', mov4.imu3.t, mov4.imu3.fb(:,1), 'g',  mov4.imu4.t, mov4.imu4.fb(:,1), 'k')
% hold on
% plot(mov4.imu1_sim.t', mov4.imu1_sim.fb(:,1), 'r', mov4.imu2_sim.t', mov4.imu2_sim.fb(:,1), 'b', mov4.imu3_sim.t', mov4.imu3_sim.fb(:,1), 'g',  mov4.imu4_sim.t', mov4.imu4_sim.fb(:,1), 'k','LineWidth',2)
% hold off
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4")
% "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])

subplot(3,1,2)
plot(mov4.imu1.t, mov4.imu1.fb(:,2), 'r', mov4.imu2.t, mov4.imu2.fb(:,2), 'b', mov4.imu3.t, mov4.imu3.fb(:,2), 'g',  mov4.imu4.t, mov4.imu4.fb(:,2), 'k')
% hold on
% plot(mov4.imu1_sim.t', mov4.imu1_sim.fb(:,2), 'r', mov4.imu2_sim.t', mov4.imu2_sim.fb(:,2), 'b', mov4.imu3_sim.t', mov4.imu3_sim.fb(:,2), 'g',  mov4.imu4_sim.t', mov4.imu4_sim.fb(:,2), 'k','LineWidth',2)
% hold off
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])

subplot(3,1,3)
plot(mov4.imu1.t, mov4.imu1.fb(:,3), 'r', mov4.imu2.t, mov4.imu2.fb(:,3), 'b', mov4.imu3.t, mov4.imu3.fb(:,3), 'g',  mov4.imu4.t, mov4.imu4.fb(:,3), 'k')
% hold on
% plot(mov4.imu1_sim.t', mov4.imu1_sim.fb(:,3), 'r', mov4.imu2_sim.t', mov4.imu2_sim.fb(:,3), 'b', mov4.imu3_sim.t', mov4.imu3_sim.fb(:,3), 'g',  mov4.imu4_sim.t', mov4.imu4_sim.fb(:,3), 'k','LineWidth',2)
% hold off
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4")
% "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])
% saveas(fh, 'rawacc_motion2.png')


fh = figure(14);

sgtitle('Gyroscope raw values. Motion 4. [$deg/s$].','interpreter','latex')

subplot(3,1,1)
plot(mov4.imu1.t, rad2deg(mov4.imu1.wb(:,1)), 'r', mov4.imu2.t, rad2deg(mov4.imu2.wb(:,1)), 'b', mov4.imu3.t, rad2deg(mov4.imu3.wb(:,1)), 'g',  mov4.imu4.t, rad2deg(mov4.imu4.wb(:,1)), 'k')
% hold on
% plot(mov4.imu1_sim.t', rad2deg(mov4.imu1_sim.wb(:,1)), 'r', mov4.imu2_sim.t', rad2deg(mov4.imu2_sim.wb(:,1)), 'b', mov4.imu3_sim.t', rad2deg(mov4.imu3_sim.wb(:,1)), 'g',  mov4.imu4_sim.t', rad2deg(mov4.imu4_sim.wb(:,1)), 'k','LineWidth',2)
% hold off
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4")
% "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])

subplot(3,1,2)
plot(mov4.imu1.t, rad2deg(mov4.imu1.wb(:,2)), 'r', mov4.imu2.t, rad2deg(mov4.imu2.wb(:,2)), 'b', mov4.imu3.t, rad2deg(mov4.imu3.wb(:,2)), 'g',  mov4.imu4.t, rad2deg(mov4.imu4.wb(:,2)), 'k')
% hold on
% plot(mov4.imu1_sim.t', rad2deg(mov4.imu1_sim.wb(:,2)), 'r', mov4.imu2_sim.t', rad2deg(mov4.imu2_sim.wb(:,2)), 'b', mov4.imu3_sim.t', rad2deg(mov4.imu3_sim.wb(:,2)), 'g',  mov4.imu4_sim.t', rad2deg(mov4.imu4_sim.wb(:,2)), 'k','LineWidth',2)
% hold off
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4")
% "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])

subplot(3,1,3)
plot(mov4.imu1.t, rad2deg(mov4.imu1.wb(:,3)), 'r', mov4.imu2.t, rad2deg(mov4.imu2.wb(:,3)), 'b', mov4.imu3.t, rad2deg(mov4.imu3.wb(:,3)), 'g',  mov4.imu4.t, rad2deg(mov4.imu4.wb(:,3)), 'k')
% hold on
% plot(mov4.imu1_sim.t', rad2deg(mov4.imu1_sim.wb(:,3)), 'r', mov4.imu2_sim.t', rad2deg(mov4.imu2_sim.wb(:,3)), 'b', mov4.imu3_sim.t', rad2deg(mov4.imu3_sim.wb(:,3)), 'g',  mov4.imu4_sim.t', rad2deg(mov4.imu4_sim.wb(:,3)), 'k','LineWidth',2)
% hold off
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4")
% "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])


% saveas(fh, 'rawgyro_motion2.png')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(15);

sgtitle('quaternions - KF. Movement 4. Architecture 1.','interpreter','latex')

subplot(4,1,1)
plot(mov4.navCM.t, mov4.navCM.qua(:,1), 'ro')
hold on
plot(mov4.navCM.t, mov4.quat(:,1), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q1','interpreter','latex')
legend('q1 KF','q1 th','interpreter','latex')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])

subplot(4,1,2)
plot(mov4.navCM.t, mov4.navCM.qua(:,2), 'ro')
hold on
plot(mov4.navCM.t, mov4.quat(:,2), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q2','interpreter','latex')
legend('q2 KF','q2 th','interpreter','latex')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])

subplot(4,1,3)
plot(mov4.navCM.t, mov4.navCM.qua(:,3), 'ro')
hold on
plot(mov4.navCM.t, mov4.quat(:,3), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q3','interpreter','latex')
legend('q3 KF','q3 th','interpreter','latex')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])

subplot(4,1,4)
plot(mov4.navCM.t, mov4.navCM.qua(:,4), 'ro')
hold on
plot(mov4.navCM.t, mov4.quat(:,4), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q4','interpreter','latex')
legend('q4 KF','q4 th','interpreter','latex')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])

% saveas(fh, 'quaternions_motion1.png')

fh = figure(16);
sgtitle('Euler angles - KF. Movement 4. Architecture 1.','interpreter','latex')

subplot(3,1,1)
plot(mov4.navCM.t, mov4.navCM.roll, 'ro')
hold on
plot(mov4.navCM.t, rad2deg(mov4.roll_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('roll [deg]','interpreter','latex')
legend('roll KF','roll th','interpreter','latex')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])

subplot(3,1,2)
plot(mov4.navCM.t, mov4.navCM.pitch, 'ro')
hold on
plot(mov4.navCM.t, rad2deg(mov4.pitch_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('pitch [deg]','interpreter','latex')
legend('pitch KF','pitch th','interpreter','latex')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])

subplot(3,1,3)
plot(mov4.navCM.t, mov4.navCM.yaw, 'ro')
hold on
plot(mov4.navCM.t, rad2deg(mov4.yaw_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('yaw [deg]','interpreter','latex')
legend('yaw KF','yaw th','interpreter','latex')
grid minor
xlim([mov4.imu1.t(1), mov4.imu1.t(end)])

% saveas(fh, 'eulerangles_motion1.png')

%% MOVEMENT 5
start_pos_mov5 = 16405;
end_pos_mov5 = 18695;

[mov5] = separate_motion(start_pos_mov5, end_pos_mov5, imu1, imu2, imu3, imu4);
mov5.imu1.t = mov5.imu1.t - mov5.imu1.t(1);
mov5.imu2.t = mov5.imu2.t - mov5.imu2.t(1);
mov5.imu3.t = mov5.imu3.t - mov5.imu3.t(1);
mov5.imu4.t = mov5.imu4.t - mov5.imu4.t(1);

mov5.imu1.ini_align = deg2rad([0, 0, 0]);
mov5.imu2.ini_align = deg2rad([0, 0, 0]);
mov5.imu3.ini_align = deg2rad([0, 0, 0]);
mov5.imu4.ini_align = deg2rad([0, 0, 0]);

[mov5.navCM, mov5.kf] = arch1_imu2cm_filter_nomag(mov5.imu1, mov5.imu2, mov5.imu3, mov5.imu4);
mov5.navCM.yaw = rad2deg(unwrap(deg2rad(mov5.navCM.yaw)));

% mov5.navCM.yaw = unwrap(mov5.navCM.yaw);
arch1_mov5_navCM = mov5.navCM;
mov5_time = mov5.imu1.t;

save('mov5_time','mov5_time');
save('arch1_mov5_navCM','arch1_mov5_navCM');

% mov5 = get_sigmas(mov5);

% simulate real motion
mov5.imuMAIN.t = mov5.imu1.t;

end_first_motion7 = 17227 - start_pos_mov5 ;
end_second_motion7 = 18274 - start_pos_mov5;
end_third_motion7 = end_pos_mov5 - start_pos_mov5 + 1;

mov5.roll_true(1:end_first_motion7) = deg2rad(0.25)*ones(length(mov5.imuMAIN.t(1:end_first_motion7)),1);
mov5.pitch_true(1:end_first_motion7) = deg2rad(0.2)*ones(length(mov5.imuMAIN.t(1:end_first_motion7)),1);
mov5.yaw_true(1:end_first_motion7) = deg2rad(50)*mov5.imuMAIN.t(1:end_first_motion7 );

mov5.roll_true(end_first_motion7:end_second_motion7) = deg2rad(0.5)*ones(length(mov5.imuMAIN.t(end_first_motion7:end_second_motion7)),1);
mov5.pitch_true(end_first_motion7:end_second_motion7) = deg2rad(0.2)*ones(length(mov5.imuMAIN.t(end_first_motion7:end_second_motion7)),1);
mov5.yaw_true(end_first_motion7:end_second_motion7) = deg2rad(100)*(mov5.imuMAIN.t(end_first_motion7:end_second_motion7) - mov5.imuMAIN.t(end_first_motion7)) + mov5.yaw_true(end_first_motion7 - 1);

mov5.roll_true(end_second_motion7:end_third_motion7) = deg2rad(1.2)*ones(length(mov5.imuMAIN.t(end_second_motion7:end_third_motion7)),1);
mov5.pitch_true(end_second_motion7:end_third_motion7) = -deg2rad(0.3)*ones(length(mov5.imuMAIN.t(end_second_motion7:end_third_motion7)),1);
mov5.yaw_true(end_second_motion7:end_third_motion7) = deg2rad(200)*(mov5.imuMAIN.t(end_second_motion7:end_third_motion7) - mov5.imuMAIN.t(end_second_motion7)) + mov5.yaw_true(end_second_motion7 - 1);

mov5.wx(1) = 0;
mov5.wy(1) = 0;
mov5.wz(1) = deg2rad(50);
mov5.dt = diff(mov5.imu1.t);

for i = 2:length(mov5.imu1.t)
    mov5.wx(i) = (mov5.roll_true(i) - mov5.roll_true(i-1))/mov5.dt(i-1);
    mov5.wy(i) = (mov5.pitch_true(i) - mov5.pitch_true(i-1))/mov5.dt(i-1);
    mov5.wz(i) = (mov5.yaw_true(i) - mov5.yaw_true(i-1))/mov5.dt(i-1);
end
% Generate synthetic motion by defining angular velocities
mov5.imuMAIN.wv(:,1) = mov5.wx;
mov5.imuMAIN.wv(:,2) = mov5.wy;
mov5.imuMAIN.wv(:,3) = mov5.wz;
mov5.imuMAIN.ini_align = deg2rad([0, 0, 0]);

% Generate synthetic motion for the acceleration and magnetic field.
[mov5.imuMAIN] = IMU_simulator(mov5.imuMAIN);
[mov5.imu1_sim] = VIMU_to_IMU(mov5.imuMAIN, mov5.imu1);
[mov5.imu2_sim] = VIMU_to_IMU(mov5.imuMAIN, mov5.imu2);
[mov5.imu3_sim] = VIMU_to_IMU(mov5.imuMAIN, mov5.imu3);
[mov5.imu4_sim] = VIMU_to_IMU(mov5.imuMAIN, mov5.imu4);


for i = 1:length(mov5.imuMAIN.t)
    euler = [mov5.roll_true(i), mov5.pitch_true(i), mov5.yaw_true(i)];
    mov5.quat(i,:) = euler2qua(euler);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RAW VALUES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(17);
fh.WindowState = 'maximized';

sgtitle('Accelerometer raw values. Motion 5. [$m/s^2$]','interpreter','latex')

subplot(3,1,1)
plot(mov5.imu1.t, mov5.imu1.fb(:,1), 'r', mov5.imu2.t, mov5.imu2.fb(:,1), 'b', mov5.imu3.t, mov5.imu3.fb(:,1), 'g',  mov5.imu4.t, mov5.imu4.fb(:,1), 'k')
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4")
legend('location','westoutside')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])

subplot(3,1,2)
plot(mov5.imu1.t, mov5.imu1.fb(:,2), 'r', mov5.imu2.t, mov5.imu2.fb(:,2), 'b', mov5.imu3.t, mov5.imu3.fb(:,2), 'g',  mov5.imu4.t, mov5.imu4.fb(:,2), 'k')
% hold on
% plot(mov5.imu1_sim.t', mov5.imu1_sim.fb(:,2), 'r', mov5.imu2_sim.t', mov5.imu2_sim.fb(:,2), 'b', mov5.imu3_sim.t', mov5.imu3_sim.fb(:,2), 'g',  mov5.imu4_sim.t', mov5.imu4_sim.fb(:,2), 'k','LineWidth',2)
% hold off
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4")
% "IMU1 th","IMU2 th","IMU3 th","IMU4 th",'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])

subplot(3,1,3)
plot(mov5.imu1.t, mov5.imu1.fb(:,3), 'r', mov5.imu2.t, mov5.imu2.fb(:,3), 'b', mov5.imu3.t, mov5.imu3.fb(:,3), 'g',  mov5.imu4.t, mov5.imu4.fb(:,3), 'k')
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4")
legend('location','westoutside')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])
% saveas(fh, 'rawacc_motion2.png')


fh = figure(18);
fh.WindowState = 'maximized';

sgtitle('Gyroscope raw values. Motion 5. [$deg/s$].','interpreter','latex')

subplot(3,1,1)
plot(mov5.imu1.t, rad2deg(mov5.imu1.wb(:,1)), 'r', mov5.imu2.t, rad2deg(mov5.imu2.wb(:,1)), 'b', mov5.imu3.t, rad2deg(mov5.imu3.wb(:,1)), 'g',  mov5.imu4.t, rad2deg(mov5.imu4.wb(:,1)), 'k')
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4")
legend('location','westoutside')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])

subplot(3,1,2)
plot(mov5.imu1.t, rad2deg(mov5.imu1.wb(:,2)), 'r', mov5.imu2.t, rad2deg(mov5.imu2.wb(:,2)), 'b', mov5.imu3.t, rad2deg(mov5.imu3.wb(:,2)), 'g',  mov5.imu4.t, rad2deg(mov5.imu4.wb(:,2)), 'k')
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4")
legend('location','westoutside')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])

subplot(3,1,3)
plot(mov5.imu1.t, rad2deg(mov5.imu1.wb(:,3)), 'r', mov5.imu2.t, rad2deg(mov5.imu2.wb(:,3)), 'b', mov5.imu3.t, rad2deg(mov5.imu3.wb(:,3)), 'g',  mov5.imu4.t, rad2deg(mov5.imu4.wb(:,3)), 'k')
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4")
legend('location','westoutside')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])


% saveas(fh, 'rawgyro_motion2.png')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure(19);
fh.WindowState = 'maximized';

sgtitle('quaternions - KF. Movement 5. Architecture 1.','interpreter','latex')

subplot(4,1,1)
plot(mov5.navCM.t, mov5.navCM.qua(:,1), 'ro')
hold on
plot(mov5.navCM.t, mov5.quat(:,1), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q1','interpreter','latex')
legend('q1 KF','q1 th','interpreter','latex')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])
ylim([-1, 1])

subplot(4,1,2)
plot(mov5.navCM.t, mov5.navCM.qua(:,2), 'ro')
hold on
plot(mov5.navCM.t, mov5.quat(:,2), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q2','interpreter','latex')
legend('q2 KF','q2 th','interpreter','latex')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])
ylim([-1, 1])

subplot(4,1,3)
plot(mov5.navCM.t, mov5.navCM.qua(:,3), 'ro')
hold on
plot(mov5.navCM.t, mov5.quat(:,3), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q3','interpreter','latex')
legend('q3 KF','q3 th','interpreter','latex')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])
ylim([-1, 1])

subplot(4,1,4)
plot(mov5.navCM.t, mov5.navCM.qua(:,4), 'ro')
hold on
plot(mov5.navCM.t, mov5.quat(:,4), 'k','LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('q4','interpreter','latex')
legend('q4 KF','q4 th','interpreter','latex')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])

% saveas(fh, 'quaternions_motion1.png')

fh = figure(20);
fh.WindowState = 'maximized';
sgtitle('Euler angles - KF. Movement 5. Architecture 1.','interpreter','latex')

subplot(3,1,1)
plot(mov5.navCM.t, mov5.navCM.roll, 'ro')
hold on
plot(mov5.navCM.t, rad2deg(mov5.roll_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('roll [deg]','interpreter','latex')
legend('roll KF','roll th','interpreter','latex')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])

subplot(3,1,2)
plot(mov5.navCM.t, mov5.navCM.pitch, 'ro')
hold on
plot(mov5.navCM.t, rad2deg(mov5.pitch_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('pitch [deg]','interpreter','latex')
legend('pitch KF','pitch th','interpreter','latex')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])

subplot(3,1,3)
plot(mov5.navCM.t, mov5.navCM.yaw, 'ro')
hold on
plot(mov5.navCM.t, rad2deg(mov5.yaw_true),'k', 'LineWidth',2)
hold off
xlabel('Time [s]','interpreter','latex')
ylabel('yaw [deg]','interpreter','latex')
legend('yaw KF','yaw th','interpreter','latex')
grid minor
xlim([mov5.imu1.t(1), mov5.imu1.t(end)])

% saveas(fh, 'eulerangles_motion1.png')


%% PLOT RAW DATA

fh = figure(21);
fh.WindowState = 'maximized';
sgtitle('Accelerometer raw values. [$m/s^2$]','interpreter','latex')

subplot(3,1,1)
plot(imu1.t, imu1.fb(:,1), 'r', imu2.t, imu2.fb(:,1), 'b', imu3.t, imu3.fb(:,1), 'g', imu4.t, imu4.fb(:,1), 'k')
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", 'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([imu1.t(1), imu1.t(end)])

subplot(3,1,2)
plot(imu1.t, imu1.fb(:,2), 'r', imu2.t, imu2.fb(:,2), 'b', imu3.t, imu3.fb(:,2), 'g', imu4.t, imu4.fb(:,2), 'k')
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", 'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([imu1.t(1), imu1.t(end)])

subplot(3,1,3)
plot(imu1.t, imu1.fb(:,3), 'r', imu2.t, imu2.fb(:,3), 'b', imu3.t, imu3.fb(:,3), 'g', imu4.t, imu4.fb(:,3), 'k')
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$a_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", 'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([imu1.t(1), imu1.t(end)])

% saveas(fh, 'rawacc.png')

fh = figure(22);
fh.WindowState = 'maximized';
sgtitle('Gyroscope raw values. [$deg/s$].','interpreter','latex')

subplot(3,1,1)
plot(imu1.t, rad2deg(imu1.wb(:,1)), 'r', imu2.t, rad2deg(imu2.wb(:,1)), 'b', imu3.t, rad2deg(imu3.wb(:,1)), 'g', imu4.t, rad2deg(imu4.wb(:,1)), 'k')
title("X coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", 'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([imu1.t(1), imu1.t(end)])

subplot(3,1,2)
plot(imu1.t, rad2deg(imu1.wb(:,2)), 'r', imu2.t, rad2deg(imu2.wb(:,2)), 'b', imu3.t, rad2deg(imu3.wb(:,2)), 'g', imu4.t, rad2deg(imu4.wb(:,2)), 'k')
title("Y coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", 'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([imu1.t(1), imu1.t(end)])

subplot(3,1,3)
plot(imu1.t, rad2deg(imu1.wb(:,3)), 'r', imu2.t, rad2deg(imu2.wb(:,3)), 'b', imu3.t, rad2deg(imu3.wb(:,3)), 'g', imu4.t, rad2deg(imu4.wb(:,3)), 'k')
title("Z coordinate", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("$\omega_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4", 'interpreter','latex')
legend('location','westoutside')
grid minor
xlim([imu1.t(1), imu1.t(end)])

% saveas(fh, 'rawgyro.png')