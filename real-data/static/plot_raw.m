clc; clear; close all;

%% Add path
addpath ../../data-acquire/data-processing/post-processing/files/
addpath ../

%% Data acquisition
test = csvread('static_tetra.csv');
[imu1_raw, imu2_raw, imu3_raw, imu4_raw] = read_realdata(test);

%% Accelerometer
imu1_raw.t = imu1_raw.t - imu1_raw.t(1);
imu2_raw.t = imu2_raw.t - imu2_raw.t(1);
imu3_raw.t = imu3_raw.t - imu3_raw.t(1);
imu4_raw.t = imu4_raw.t - imu4_raw.t(1);

save('imu1_raw.mat','imu1_raw');
save('imu2_raw.mat','imu2_raw');
save('imu3_raw.mat','imu3_raw');
save('imu4_raw.mat','imu4_raw');

fh = figure(1);
fh.WindowState = 'maximized';

imu1_raw.t = imu1_raw.t - imu1_raw.t(1);
imu2_raw.t = imu1_raw.t - imu1_raw.t(1);
imu3_raw.t = imu1_raw.t - imu1_raw.t(1);
imu4_raw.t = imu1_raw.t - imu1_raw.t(1);
sgtitle('Accelerometer raw values [$m/s^2$]','interpreter','latex')

subplot(3,1,1)
plot(imu1_raw.t, imu1_raw.fb(:,1), 'r', imu2_raw.t, imu2_raw.fb(:,1), 'b', imu3_raw.t, imu3_raw.fb(:,1), 'g', imu4_raw.t, imu4_raw.fb(:,1), 'k')
title("X coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$a_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best")
legend('location','westoutside')
grid minor

subplot(3,1,2)
plot(imu1_raw.t, imu1_raw.fb(:,2), 'r', imu2_raw.t, imu2_raw.fb(:,2), 'b', imu3_raw.t, imu3_raw.fb(:,2), 'g', imu4_raw.t, imu4_raw.fb(:,2), 'k')
title("Y coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$a_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best")
legend('location','westoutside')
grid minor

subplot(3,1,3)
plot(imu1_raw.t, imu1_raw.fb(:,3), 'r', imu2_raw.t, imu2_raw.fb(:,3), 'b', imu3_raw.t, imu3_raw.fb(:,3), 'g', imu4_raw.t, imu4_raw.fb(:,3), 'k')
title("Z coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$a_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best")
legend('location','westoutside')
grid minor

%% Gyroscope

fh = figure(2);
fh.WindowState = 'maximized';


sgtitle('Gyroscope raw values [$deg/s$]','interpreter','latex')

subplot(3,1,1)
plot(imu1_raw.t, imu1_raw.wb(:,1), 'r', imu2_raw.t, imu2_raw.wb(:,1), 'b', imu3_raw.t, imu3_raw.wb(:,1), 'g', imu4_raw.t, imu4_raw.wb(:,1), 'k')
title("X coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$\omega_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best")
legend('location','westoutside')
grid minor

subplot(3,1,2)
plot(imu1_raw.t, imu1_raw.wb(:,2), 'r', imu2_raw.t, imu2_raw.wb(:,2), 'b', imu3_raw.t, imu3_raw.wb(:,2), 'g', imu4_raw.t, imu4_raw.wb(:,2), 'k')
title("Y coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$\omega_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best")
legend('location','westoutside')
grid minor

subplot(3,1,3)
plot(imu1_raw.t, imu1_raw.wb(:,3), 'r', imu2_raw.t, imu2_raw.wb(:,3), 'b', imu3_raw.t, imu3_raw.wb(:,3), 'g', imu4_raw.t, imu4_raw.wb(:,3), 'k')
title("Z coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$\omega_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best")
legend('location','westoutside')
grid minor

