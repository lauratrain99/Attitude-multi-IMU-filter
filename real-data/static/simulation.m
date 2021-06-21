clc;clear;close all;

%% Add path
addpath ../../synthetic-data/simulation
addpath ../../synthetic-data/simulation/imu2cm
addpath ../../conversions
addpath ../../ins

%% load raw values
load('imu1_raw.mat')
load('imu2_raw.mat')
load('imu3_raw.mat')
load('imu4_raw.mat')

%% SIMULATE MOTION FOR THE BODY (CENTER OF MASS)

% Initial considerations:
% B, body reference frame of the each of the IMUs
% V, body reference frame of the center of mass of the multi-IMU body
% DCMvb is the DCM from the ith-imu body reference frame to the center of
% mass of body. It is fixed and known a priori. 
% imuMAIN, the synthetic motion of the center of mass

t = 0:10:967.975;
imuMAIN.t = t;
N = length(imuMAIN.t);
imuMAIN.ini_align = [0, 0, 0];

% Generate synthetic motion by defining angular velocities
w = 0;
imuMAIN.wv(:,1) = zeros(N, 1);
imuMAIN.wv(:,2) = zeros(N, 1);
imuMAIN.wv(:,3) = zeros(N, 1);

% Generate synthetic motion for the acceleration and magnetic field.
[imuMAIN] = IMU_simulator(imuMAIN);

%% GENERATE DATA FOR IMU NUMBER 1
% imu1, imu located at a point different from the center of mass
% imu1MAINcomputed, the data for the center of mass obtained from the IMU1

% length from the center of mass to the IMU1
L = 0.065;

% relative orientation of the IMU1 with respect to the center of mass of
% the body
imu1.ini_align = [0, 0, 0];

imu1.roll0 = 0;
imu1.pitch0 = 0;
imu1.yaw0 = 0;


% DCM from IMU1 body reference frame to the VIMU ref frame
imu1.DCMvb =  euler2dcm(deg2rad([imu1.roll0, imu1.pitch0, imu1.yaw0]));
imu1.DCMbv = imu1.DCMvb';

% lever arm in VIMU coordinates
imu1.Rvb = imu1.DCMbv*[0, 0, L]';

% convert the synthetic data of the body into data for the IMU1
[imu1] = VIMU_to_IMU(imuMAIN, imu1);


%% GENERATE DATA FOR IMU NUMBER 2
% imu2, imu located at a point different from the center of mass
% imu2MAINcomputed, the data for the center of mass obtained from the IMU2

% length from the center of mass to the IMU2
L = 0.065;

% relative orientation of the IMU1 with respect to the center of mass of
% the body
imu2.ini_align = [0, 0, 0];

imu2.roll0 = 0;
imu2.pitch0 = 120;
imu2.yaw0 = 45;


% DCM from IMU2 body reference frame to the VIMU ref frame
imu2.DCMvb =  euler2dcm(deg2rad([imu2.roll0, imu2.pitch0, imu2.yaw0]));
imu2.DCMbv = imu2.DCMvb';

% lever arm in VIMU coordinates
imu2.Rvb = imu2.DCMbv*[0, 0, L]';

% convert the synthetic data of the body into data for the IMU2
[imu2] = VIMU_to_IMU(imuMAIN, imu2);

%% GENERATE DATA FOR IMU NUMBER 3
% imu3, imu located at a point different from the center of mass
% imu3MAINcomputed, the data for the center of mass obtained from the IMU3

% length from the center of mass to the IMU3
L = 0.065;

% relative orientation of the IMU3 with respect to the center of mass of
% the body
imu3.ini_align = [0, 0, 0];

imu3.roll0 = 0;
imu3.pitch0 = 120;
imu3.yaw0 = 180;



% DCM from IMU3 body reference frame to the VIMU ref frame
imu3.DCMvb =  euler2dcm(deg2rad([imu3.roll0, imu3.pitch0, imu3.yaw0]));
imu3.DCMbv = imu3.DCMvb';

% lever arm in VIMU coordinates
imu3.Rvb = imu3.DCMbv*[0, 0, L]';

% convert the synthetic data of the body into data for the IMU3
[imu3] = VIMU_to_IMU(imuMAIN, imu3);


%% GENERATE DATA FOR IMU NUMBER 4
% imu4, imu located at a point different from the center of mass
% imu4MAINcomputed, the data for the center of mass obtained from the IMU4

% length from the center of mass to the IMU4
L = 0.065;

% relative orientation of the IMU4 with respect to the center of mass of
% the body
imu4.ini_align = [0, 0, 0];

imu4.roll0 = 0;
imu4.pitch0 = 120;
imu4.yaw0 = -45;


% DCM from IMU4 body reference frame to the VIMU ref frame
imu4.DCMvb =  euler2dcm(deg2rad([imu4.roll0, imu4.pitch0, imu4.yaw0]));
imu4.DCMbv = imu4.DCMvb';

% lever arm in VIMU coordinates
imu4.Rvb = imu4.DCMbv*[0, 0, L]';

% convert the synthetic data of the body into data for the IMU3
[imu4] = VIMU_to_IMU(imuMAIN, imu4);


%%

fh = figure(4);
fh.WindowState = 'maximized';


sgtitle('Accelerometer raw values [$m/s^2$]','interpreter','latex')

subplot(3,1,1)
plot(imu1.t, imu1.fb(:,1), 'ro', imu2.t, imu2.fb(:,1), 'bo', imu3.t, imu3.fb(:,1), 'go', imu4.t, imu4.fb(:,1), 'ko', ...
    imu1_raw.t, imu1_raw.fb(:,1), 'r', imu2_raw.t, imu2_raw.fb(:,1), 'b', imu3_raw.t, imu3_raw.fb(:,1), 'g', imu4_raw.t, imu4_raw.fb(:,1), 'k')
title("X coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$a_x$", 'interpreter','latex')
legend("IMU1 th","IMU2 th","IMU3 th","IMU4 th","IMU1 raw","IMU2 raw","IMU3 raw","IMU4 raw" )
legend('location','westoutside')
grid minor

subplot(3,1,2)
plot(imu1.t, imu1.fb(:,2), 'ro', imu2.t, imu2.fb(:,2), 'bo', imu3.t, imu3.fb(:,2), 'go', imu4.t, imu4.fb(:,2), 'ko', ...
    imu1_raw.t, imu1_raw.fb(:,2), 'r', imu2_raw.t, imu2_raw.fb(:,2), 'b', imu3_raw.t, imu3_raw.fb(:,2), 'g', imu4_raw.t, imu4_raw.fb(:,2), 'k')
title("Y coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$a_y$", 'interpreter','latex')
legend("IMU1 th","IMU2 th","IMU3 th","IMU4 th","IMU1 raw","IMU2 raw","IMU3 raw","IMU4 raw" )
legend('location','westoutside')
grid minor

subplot(3,1,3)
plot(imu1.t, imu1.fb(:,3), 'ro', imu2.t, imu2.fb(:,3), 'bo', imu3.t, imu3.fb(:,3), 'go', imu4.t, imu4.fb(:,3), 'ko', ...
    imu1_raw.t, imu1_raw.fb(:,3), 'r', imu2_raw.t, imu2_raw.fb(:,3), 'b', imu3_raw.t, imu3_raw.fb(:,3), 'g', imu4_raw.t, imu4_raw.fb(:,3), 'k')
title("Z coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$a_z$", 'interpreter','latex')
legend("IMU1 th","IMU2 th","IMU3 th","IMU4 th","IMU1 raw","IMU2 raw","IMU3 raw","IMU4 raw" )
legend('location','westoutside')
grid minor

%% Gyroscope

fh = figure(3);
fh.WindowState = 'maximized';


sgtitle('Gyroscope raw values [$deg/s$]','interpreter','latex')

subplot(3,1,1)
plot(imu1.t, imu1.wb(:,1), 'ro', imu2.t, imu2.wb(:,1), 'bo', imu3.t, imu3.wb(:,1), 'go', imu4.t, imu4.wb(:,1), 'ko', ...
    imu1_raw.t, imu1_raw.wb(:,1), 'r', imu2_raw.t, imu2_raw.wb(:,1), 'b', imu3_raw.t, imu3_raw.wb(:,1), 'g', imu4_raw.t, imu4_raw.wb(:,1), 'k')
title("X coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$\omega_x$", 'interpreter','latex')
legend("IMU1 th","IMU2 th","IMU3 th","IMU4 th","IMU1 raw","IMU2 raw","IMU3 raw","IMU4 raw" )
legend('location','westoutside')
grid minor

subplot(3,1,2)
plot(imu1.t, imu1.wb(:,2), 'ro', imu2.t, imu2.wb(:,2), 'bo', imu3.t, imu3.wb(:,2), 'go', imu4.t, imu4.wb(:,2), 'ko', ...
    imu1_raw.t, imu1_raw.wb(:,2), 'r', imu2_raw.t, imu2_raw.wb(:,2), 'b', imu3_raw.t, imu3_raw.wb(:,2), 'g', imu4_raw.t, imu4_raw.wb(:,2), 'k')
title("Y coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$\omega_y$", 'interpreter','latex')
legend("IMU1 th","IMU2 th","IMU3 th","IMU4 th","IMU1 raw","IMU2 raw","IMU3 raw","IMU4 raw" )
legend('location','westoutside')
grid minor

subplot(3,1,3)
plot(imu1.t, imu1.wb(:,3), 'ro', imu2.t, imu2.wb(:,3), 'bo', imu3.t, imu3.wb(:,3), 'go', imu4.t, imu4.wb(:,3), 'ko', ...
    imu1_raw.t, imu1_raw.wb(:,3), 'r', imu2_raw.t, imu2_raw.wb(:,3), 'b', imu3_raw.t, imu3_raw.wb(:,3), 'g', imu4_raw.t, imu4_raw.wb(:,3), 'k')
title("Z coordinate", 'interpreter','latex')
xlabel("Time [s]", 'interpreter','latex')
ylabel("$\omega_z$", 'interpreter','latex')
legend("IMU1 th","IMU2 th","IMU3 th","IMU4 th","IMU1 raw","IMU2 raw","IMU3 raw","IMU4 raw" )
legend('location','westoutside')
grid minor