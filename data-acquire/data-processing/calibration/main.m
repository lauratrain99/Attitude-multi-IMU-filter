% This script processes data and performs analysis of measured signal in
% terms of power spectral density and Allan variance
%   Written By: David Hanley (9/28/2016)
clear; clc; close all;


%% Add paths
addpath ../post-processing/
addpath ../post-processing/files/

%% Load data
test = csvread('night_multi.csv');

[imu1, imu2, imu3, imu4] = read_4imu(test);

%% Data Loading 

imu1_acc.x = 0;
imu1_acc.y = 0;
imu1_acc.z = 9.8;

imu234_acc.x = -9.81*cosd(30);
imu234_acc.y = 0;
imu234_acc.z = -9.81*sind(30);

[imu1] = calibrate_imu(imu1,imu1_acc);
[imu2] = calibrate_imu(imu2,imu234_acc);
[imu3] = calibrate_imu(imu3,imu234_acc);
[imu4] = calibrate_imu(imu4,imu234_acc);


%% Accelerometer

[imu1_error_multi] = store_errors(imu1);
[imu2_error_multi] = store_errors(imu2);
[imu3_error_multi] = store_errors(imu3);
[imu4_error_multi] = store_errors(imu4);


save('imu1_error_multi.mat','imu1_error_multi');
save('imu2_error_multi.mat','imu2_error_multi');
save('imu3_error_multi.mat','imu3_error_multi');
save('imu4_error_multi.mat','imu4_error_multi');

%%
figure(1)
plot(imu1.t, imu1.accX.meas - imu1.accX.offset, 'r', imu2.t, imu2.accX.meas - imu2.accX.offset, 'b', imu3.t, imu3.accX.meas - imu3.accX.offset, 'g', imu4.t, imu4.accX.meas - imu4.accX.offset, 'k')
title("Accelerometer X coordinate [$m/s^2$]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Acceleration [m/s^2]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best", 'interpreter','latex')
grid minor

figure(2)
plot(imu1.t, imu1.accY.meas - imu1.accY.offset, 'r', imu2.t, imu2.accY.meas - imu2.accY.offset, 'b', imu3.t, imu3.accY.meas - imu3.accY.offset, 'g', imu4.t, imu4.accY.meas - imu4.accY.offset, 'k')
title("Accelerometer Y coordinate [$m/s^2$]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Acceleration [m/s^2]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best", 'interpreter','latex')
grid minor

figure(3)
plot(imu1.t, imu1.accZ.meas - imu1.accZ.offset, 'r', imu2.t, imu2.accZ.meas - imu2.accZ.offset, 'b', imu3.t, imu3.accZ.meas - imu3.accZ.offset, 'g', imu4.t, imu4.accZ.meas - imu4.accZ.offset, 'k')
title("Accelerometer Z coordinate [$m/s^2$]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Acceleration [m/s^2]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best", 'interpreter','latex')
grid minor

%% Gyroscope

figure(4)
plot(imu1.t, imu1.gyroX.meas - imu1.gyroX.offset, 'r', imu2.t, imu2.gyroX.meas - imu2.gyroX.offset, 'b', imu3.t, imu3.gyroX.meas - imu3.gyroX.offset, 'g', imu4.t, imu4.gyroX.meas - imu4.gyroX.offset, 'k')
title("Gyroscope X coordinate [rad/s]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Angular velocity [rad/s]", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best", 'interpreter','latex')
grid minor

figure(5)
plot(imu1.t, imu1.gyroY.meas - imu1.gyroY.offset, 'r', imu2.t, imu2.gyroY.meas - imu2.gyroY.offset, 'b', imu3.t, imu3.gyroY.meas - imu3.gyroY.offset, 'g', imu4.t, imu4.gyroY.meas - imu4.gyroY.offset, 'k')
title("Gyroscope Y coordinate [rad/s]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Angular velocity [rad/s]", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best", 'interpreter','latex')
grid minor

figure(6)
plot(imu1.t, imu1.gyroZ.meas - imu1.gyroZ.offset, 'r', imu2.t, imu2.gyroZ.meas - imu2.gyroZ.offset, 'b', imu3.t, imu3.gyroZ.meas - imu3.gyroZ.offset, 'g', imu4.t, imu4.gyroZ.meas - imu4.gyroZ.offset, 'k')
title("Gyroscope Z coordinate [rad/s]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Angular velocity [rad/s]", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best", 'interpreter','latex')
grid minor

%% Allan variance accelerometer

figure(7)
loglog(imu1.accX.T, imu1.accX.sigma.^2, 'r', imu2.accX.T, imu2.accX.sigma.^2, 'b', imu3.accX.T, imu3.accX.sigma.^2, 'g', imu4.accX.T, imu4.accX.sigma.^2, 'k')
title("Allan variance accelerometer X coordinate [$m/s^2$]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Allan variance acceleration [$m/s^2$]", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best", 'interpreter','latex')
grid minor
saveas(7,"allan_accX.png")


figure(8)
loglog(imu1.accY.T, imu1.accY.sigma.^2, 'r', imu2.accY.T, imu2.accY.sigma.^2, 'b', imu3.accY.T, imu3.accY.sigma.^2, 'g', imu4.accX.T, imu4.accX.sigma.^2, 'k')
title("Allan variance accelerometer Y coordinate [$m/s^2$]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Allan variance acceleration [$m/s^2$]", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best", 'interpreter','latex')
grid minor
saveas(8,"allan_accY.png")


figure(9)
loglog(imu1.accZ.T, imu1.accZ.sigma.^2, 'r', imu2.accZ.T, imu2.accZ.sigma.^2, 'b', imu3.accZ.T, imu3.accZ.sigma.^2, 'g', imu4.accZ.T, imu4.accZ.sigma.^2, 'k')
title("Allan variance accelerometer Z coordinate [$m/s^2$]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Allan variance acceleration [$m/s^2$]", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best", 'interpreter','latex')
grid minor
saveas(9,"allan_accZ.png")


%% Allan variance gyroscope

figure(10)
loglog(imu1.gyroX.T, imu1.gyroX.sigma.^2, 'r', imu2.gyroX.T, imu2.gyroX.sigma.^2, 'b', imu3.gyroX.T, imu3.gyroX.sigma.^2, 'g', imu4.gyroX.T, imu4.gyroX.sigma.^2, 'k')
title("Allan variance gyroscope X coordinate [rad/s]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Allan variance angular velocity [rad/s]", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best", 'interpreter','latex')
grid minor
saveas(10,"allan_gyroX.png")


figure(11)
loglog(imu1.gyroY.T, imu1.gyroY.sigma.^2, 'r', imu2.gyroY.T, imu2.gyroY.sigma.^2, 'b', imu3.gyroY.T, imu3.gyroY.sigma.^2, 'g', imu4.gyroX.T, imu4.gyroX.sigma.^2, 'k')
title("Allan variance gyroscope Y coordinate [rad/s]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Allan variance angular velocity [rad/s]", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best", 'interpreter','latex')
grid minor
saveas(11,"allan_gyroY.png")


figure(12)
loglog(imu1.gyroZ.T, imu1.gyroZ.sigma.^2, 'r', imu2.gyroZ.T, imu2.gyroZ.sigma.^2, 'b', imu3.gyroZ.T, imu3.gyroZ.sigma.^2, 'g', imu4.gyroZ.T, imu4.gyroZ.sigma.^2, 'k')
title("Allan variance gyroscope Z coordinate [rad/s]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Allan variance angular velocity [rad/s]", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best", 'interpreter','latex')
grid minor
saveas(12,"allan_gyroZ.png")
