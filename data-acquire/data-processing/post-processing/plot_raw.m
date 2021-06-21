clc; clear; close all;

%% Add path
addpath files/

%% Data acquisition
test = csvread('tuesdaynight.csv');
[imu1, imu2, imu3, imu4] = read_4imu(test);
%%
imu1.t = imu1.t - imu1.t(1);
imu2.t = imu2.t - imu2.t(1);
imu3.t = imu3.t - imu3.t(1);
imu4.t = imu4.t - imu4.t(1);

%% Accelerometer

fh = figure(1);
fh.WindowState = 'maximized';


sgtitle('Accelerometer raw values [$m/s^2$]','interpreter','latex')

subplot(3,1,1)
plot(imu1.t, imu1.accX.meas, 'r', imu2.t, imu2.accX.meas, 'b', imu3.t, imu3.accX.meas, 'g', imu4.t, imu4.accX.meas, 'k')
xlim([0,imu1.t(end)])
title("X coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$a_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best")
legend('location','westoutside')
grid minor

subplot(3,1,2)
plot(imu1.t, imu1.accY.meas, 'r', imu2.t, imu2.accY.meas, 'b', imu3.t, imu3.accY.meas, 'g', imu4.t, imu4.accY.meas, 'k')
xlim([0,imu1.t(end)])
title("Y coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$a_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best")
legend('location','westoutside')
grid minor

subplot(3,1,3)
plot(imu1.t, imu1.accZ.meas, 'r', imu2.t, imu2.accZ.meas, 'b', imu3.t, imu3.accZ.meas, 'g', imu4.t, imu4.accZ.meas, 'k')
xlim([0,imu1.t(end)])
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
plot(imu1.t, imu1.gyroX.meas, 'r', imu2.t, imu2.gyroX.meas, 'b', imu3.t, imu3.gyroX.meas, 'g', imu4.t, imu4.gyroX.meas, 'k')
xlim([0,imu1.t(end)])
ylim([-0.06, 0.06])
title("X coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$\omega_x$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best")
legend('location','westoutside')
grid minor

subplot(3,1,2)
plot(imu1.t, imu1.gyroY.meas, 'r', imu2.t, imu2.gyroY.meas, 'b', imu3.t, imu3.gyroY.meas, 'g', imu4.t, imu4.gyroY.meas, 'k')
xlim([0,imu1.t(end)])
ylim([-0.06, 0.06])
title("Y coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$\omega_y$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best")
legend('location','westoutside')
grid minor

subplot(3,1,3)
plot(imu1.t, imu1.gyroZ.meas, 'r', imu2.t, imu2.gyroZ.meas, 'b', imu3.t, imu3.gyroZ.meas, 'g', imu4.t, imu4.gyroZ.meas, 'k')
xlim([0,imu1.t(end)])
ylim([-0.06, 0.06])
title("Z coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$\omega_z$", 'interpreter','latex')
legend("IMU1","IMU2","IMU3","IMU4","location","best")
legend('location','westoutside')
grid minor

%% Magnetometer

figure(7)
plot(imu1.t, imu1.magX.meas, 'r', imu2.t, imu2.magX.meas, 'b', imu3.t, imu3.magX.meas, 'g', imu4.t, imu4.magX.meas, 'k')
title("Magnetometer X coordinate [$\mu$T]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Magnetic field [$\mu$T]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best")
grid minor

figure(8)
plot(imu1.t, imu1.magY.meas, 'r', imu2.t, imu2.magY.meas, 'b', imu3.t, imu3.magY.meas, 'g', imu4.t, imu4.magY.meas, 'k')
title("Magnetometer Y coordinate [$\mu$T]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Magnetic field [$\mu$T]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best")
grid minor

figure(9)
plot(imu1.t, imu1.magZ.meas, 'r', imu2.t, imu2.magZ.meas, 'b', imu3.t, imu3.magZ.meas, 'g', imu4.t, imu4.magZ.meas, 'k')
title("Magnetometer Z coordinate [$\mu$T]", 'interpreter','latex')
xlabel("Sample time [s]", 'interpreter','latex')
ylabel("Magnetic field [$\mu$T]", 'interpreter','latex')
legend("imu1","imu2","imu3","imu4","location","best")
grid minor