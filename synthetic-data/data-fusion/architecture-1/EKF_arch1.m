%% MULTIPLE IMU EKF 
% Author: Laura Train
% Date of the last update Jan 22 2021
%
% The goal of this script is to implement the EKF with the multi-IMU
% configuration to estimate attitude generating synthetic data for a known
% simple motion. This program has to be implemented after the test in
% multiple_imu is passed.
%
% Possible design architectures:
% 1. Convert the data from each IMU to generate one CM (center of mass) data per IMU.
%    Perform the sensor fusion by averaging all the readings at the center
%    of mass. Perform the EKF only once with those readings.
% 2. Convert the data from each IMU to generate one CM (center of mass) data per IMU.
%    Perform the EKF four times, one for each of the CM readings.
%
% (To be continued with other alternatives)
%
% Checklist:
% DONE:
%          1. Obtain the data readings for each IMU.
%           
%          2. Include error characterization and consequent noise in the
%             IMU readings.
%
%          3. Build the first design architecture. Steps:
%               - Convert the data from each IMU to generate angular
%               velocity, accelerations and local magnetic field at the
%               center of mass.
%               - Perform the sensor fusion to obtain an average for the
%               angular velocity, acceleration and local magnetic field at
%               the center of mass.
%               - Apply the EKF to the averaged values.
%
%          4. Build the second design achitecture. Steps:
%               - Convert the data from each IMU to generate angular
%               velocity, accelerations and local magnetic field at the
%               center of mass.
%               - Apply the EKF to the each CM readings. Four EKFs in total

%% Use NaveGo functions
matlabrc

addpath ../../simulation
addpath ../../ins/
addpath ../../conversions/
addpath ../../kalman/

t = 0:1/100:10;

%% SIMULATE MOTION FOR THE BODY (CENTER OF MASS)

% Initial considerations:
% B, body reference frame of the each of the IMUs
% V, body reference frame of the center of mass of the multi-IMU body
% DCMvb is the DCM from the ith-imu body reference frame to the center of
% mass of body. It is fixed and known a priori. 
% imuMAIN, the synthetic motion of the center of mass

imuMAIN.t = t;
N = length(imuMAIN.t);
imuMAIN.ini_align = [0, 0, 0];

% Generate synthetic motion by defining angular velocities
w = 4*pi/10;
imuMAIN.wv(:,1) = w*ones(N,1);
imuMAIN.wv(:,2) = zeros(N, 1);
imuMAIN.wv(:,3) = zeros(N,1);

% Generate synthetic motion for the acceleration and magnetic field.
[imuMAIN] = IMU_simulator(imuMAIN);


%% GENERATE DATA FOR IMU NUMBER 1
% imu1, imu located at a point different from the center of mass
% imu1MAINcomputed, the data for the center of mass obtained from the IMU1

% length from the center of mass to the IMU1
L = 0.02;

% relative orientation of the IMU1 with respect to the center of mass of
% the body
imu1.ini_align = [0, 0, 0];
imu1.roll0 = 0;
imu1.pitch0 = 0;
imu1.yaw0 = 0;

% DCM from IMU1 body reference frame to the VIMU ref frame
imu1.DCMbv =  euler2dcm(deg2rad([imu1.roll0, imu1.pitch0, imu1.yaw0]));
imu1.DCMvb = imu1.DCMbv';

% lever arm in VIMU coordinates
imu1.Rvb = imu1.DCMvb*[0, 0, L]';

% Initialize a priori data from the sensor
imu1.ini_align = [0, 0, 0];
imu1.ini_align_err = deg2rad([0.5, 0.5, 0.5]);
imu1.gb_dyn = [0.0001, 0.0001, 0.0001];
imu1.a_std = [0.05, 0.05, 0.05];
imu1.g_std = [0.01, 0.01, 0.01];
imu1.m_std = [0.005, 0.005, 0.005];

% convert the synthetic data of the body into data for the IMU1
[imu1] = VIMU_to_IMU(imuMAIN, imu1);

% add noise to the imu sensor
imu1.wb = imu1.wb + imu1.g_std.*randn(N,3);
imu1.fb = imu1.fb + imu1.a_std.*randn(N,3);
imu1.mb = imu1.mb + imu1.m_std.*randn(N,3);



%% GENERATE DATA FOR IMU NUMBER 2
% imu2, imu located at a point different from the center of mass
% imu2MAINcomputed, the data for the center of mass obtained from the IMU2

% length from the center of mass to the IMU2
L = 0.02;

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


% Initialize a priori data from the sensor
imu2.ini_align = [0, 0, 0];
imu2.ini_align_err = deg2rad([0.5, 0.5, 0.5]);
imu2.gb_dyn = [0.0001, 0.0001, 0.0001];
imu2.a_std = [0.05, 0.05, 0.05];
imu2.g_std = [0.01, 0.01, 0.01];
imu2.m_std = [0.005, 0.005, 0.005];

% add noise to the imu sensor
imu2.wb = imu2.wb + imu2.g_std.*randn(N,3);
imu2.fb = imu2.fb + imu2.a_std.*randn(N,3);
imu2.mb = imu2.mb + imu2.m_std.*randn(N,3);


%% GENERATE DATA FOR IMU NUMBER 3
% imu3, imu located at a point different from the center of mass
% imu3MAINcomputed, the data for the center of mass obtained from the IMU3

% length from the center of mass to the IMU3
L = 0.02;

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


% Initialize a priori data from the sensor
imu3.ini_align = [0, 0, 0];
imu3.ini_align_err = deg2rad([0.5, 0.5, 0.5]);
imu3.gb_dyn = [0.0001, 0.0001, 0.0001];
imu3.a_std = [0.05, 0.05, 0.05];
imu3.g_std = [0.01, 0.01, 0.01];
imu3.m_std = [0.005, 0.005, 0.005];


%% GENERATE DATA FOR IMU NUMBER 4
% imu4, imu located at a point different from the center of mass
% imu4MAINcomputed, the data for the center of mass obtained from the IMU4

% length from the center of mass to the IMU4
L = 0.02;

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


% Initialize a priori data from the sensor
imu4.ini_align = [0, 0, 0];
imu4.ini_align_err = deg2rad([0.5, 0.5, 0.5]);
imu4.gb_dyn = [0.0001, 0.0001, 0.0001];
imu4.a_std = [0.05, 0.05, 0.05];
imu4.g_std = [0.01, 0.01, 0.01];
imu4.m_std = [0.005, 0.005, 0.005];


%% SENSOR FUSION - EXTENDED KALMAN FILTER
% Attitude EKF
[navimu1MAINcomputed , imu1MAINcomputed] = multiple_imu_filter(imu1, imu2, imu3, imu4);

%% PLOT RESULTS
% Compute attitude only using angular velocity data
imuMAIN.ini_align = [0, 0, 0];
[quat, euler] = attitude_computer(imuMAIN);
euler = rad2deg(euler);


% Plot comparison between true attitude vs EKF output
figure(1)
plot(navimu1MAINcomputed.t, quat(:,1), 'r', navimu1MAINcomputed.t, quat(:,2), 'c', navimu1MAINcomputed.t, quat(:,3), 'g', navimu1MAINcomputed.t, quat(:,4), 'k', ...
     navimu1MAINcomputed.t, navimu1MAINcomputed.qua(:,1), 'or', navimu1MAINcomputed.t, navimu1MAINcomputed.qua(:,2), 'oc', navimu1MAINcomputed.t, navimu1MAINcomputed.qua(:,3), 'og', navimu1MAINcomputed.t, navimu1MAINcomputed.qua(:,4), 'ok')
xlabel('Time [s]')
ylabel('quaternions')
legend('q1', 'q2,', 'q3', 'q4', 'q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions. Architecture 1.')
legend('location','southeast')


figure(2)
plot(navimu1MAINcomputed.t, euler(:,1), 'r', navimu1MAINcomputed.t, euler(:,2), 'c', navimu1MAINcomputed.t, euler(:,3), 'g', ...
     navimu1MAINcomputed.t, navimu1MAINcomputed.roll, 'or', navimu1MAINcomputed.t, navimu1MAINcomputed.pitch, 'oc', navimu1MAINcomputed.t, navimu1MAINcomputed.yaw, 'og')
xlabel('Time [s]')
ylabel('Euler angles [deg]')
legend('roll', 'pitch,', 'yaw', 'roll Kalman','pitch Kalman','yaw Kalman')
grid minor
title('Attitude computer vs Kalman filter. Euler angles. Architecture 2.')
legend('location','southeast')

