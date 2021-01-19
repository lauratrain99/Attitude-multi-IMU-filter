%% ATTITUDE COMPUTATION
% Author: Laura Train
% Date of the last update Jan 19 2021
%
% Checklist:
% DONE:
%          1. Input a motion of the body defined by the magnitudes of the angular
%             velocities.
%          2. Generate synthetic data (acc + magn) for the center of mass of the body.
%          3. Number of IMUs: 4. Define the orientation of each of them 
%             with respect to the center of mass by the variables roll0, pitch0, yaw0.
%          4. Convert the synthetic gyro, accel and magn data of the center
%             of mass into gyro, accel and magn data of the body reference.
%             Add noise for the gyro, accel and magn data.
%          5. Transform gyro, accel and magn data of each of the IMUs into
%             the data of the center of mass of the body.
%          6. Plot the data from the initial synthetic data to see if it
%             coincides with the transformed data.
%          7. Apply the EKF to each of the resultling center of mass data
%             coming from each of the IMUs. Results: four different
%             attitudes coming from each IMU data.
% TO DO:
%          8. After step 5, combine/fuse the center of mass data obtained
%          from each IMU and then apply the EKF with only one center of mass data


%% Use NaveGo functions
matlabrc

addpath ../ins/
addpath ../conversions/
addpath ../kalman/

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


% Generate synthetic motion by defining angular velocities
imuMAIN.wv(:,1) = 4*pi/10*ones(N,1) + imuMAIN.g_std(1)*randn(N,1);
imuMAIN.wv(:,2) = zeros(N, 1) + imuMAIN.g_std(2)*randn(N,1);
imuMAIN.wv(:,3) = zeros(N, 1) + imuMAIN.g_std(3)*randn(N,1);

% Generate synthetic motion for the acceleration and magnetic field.
[imuMAIN] = IMU_simulator(imuMAIN);


%% GENERATE DATA FOR IMU NUMBER 1
% imu1, the synthetic motion of the center of mass
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

% convert the synthetic data of the body into data for the IMU1
[imu1] = VIMU_to_IMU(imuMAIN, imu1);

% convert the data of the IMU1 back to the center of mass
[imu1MAINcomputed] = IMU_to_VIMU(imuMAIN);

% Initialize a priori data from the sensor
imu1MAINcomputed.ini_align = [0, 0, 0];
imu1MAINcomputed.ini_align_err = deg2rad([0.5, 0.5, 0.5]);
imu1MAINcomputed.gb_dyn = [0.0001, 0.0001, 0.0001];
imu1MAINcomputed.a_std = [0.05, 0.05, 0.05];
imu1MAINcomputed.g_std = [0.01, 0.01, 0.01];
imu1MAINcomputed.m_std = [0.005, 0.005, 0.005];

imu1MAINcomputed.wb = imu1MAINcomputed.wv;
imu1MAINcomputed.fb = imu1MAINcomputed.fv;
imu1MAINcomputed.mb = imu1MAINcomputed.mv;


[navimu1MAINcomputed , kfimu1MAINcomputed] = imu_filter_magnetometer(imu1MAINcomputed);


figure(1)
plot(imuMAIN.t, imuMAIN.wv(:,1), 'b', imu1MAINcomputed.t, imu1MAINcomputed.wv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.wv(:,2), 'r', imu1MAINcomputed.t, imu1MAINcomputed.wv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.wv(:,3), 'g', imu1MAINcomputed.t, imu1MAINcomputed.wv(:,3), '*g')
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity of the center of mass of the multi-IMU system over time from IMU 1')
legend('wbx', 'wbx computed', 'wby', 'wby computed', 'wbz', 'wbz computed')
grid minor

figure(2)
plot(imuMAIN.t, imuMAIN.fv(:,1), 'b', imu1MAINcomputed.t, imu1MAINcomputed.fv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.fv(:,2), 'r', imu1MAINcomputed.t, imu1MAINcomputed.fv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.fv(:,3), 'g', imu1MAINcomputed.t, imu1MAINcomputed.fv(:,3), '*g')
xlabel('Time [s]')
ylabel('Acceleration [m/^2]')
title('Acceleration of the center of mass of the multi-IMU system over time from IMU 1')
legend('abx', 'abx computed', 'aby', 'aby computed', 'abz', 'abz computed')
grid minor

figure(3)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu1MAINcomputed.t, imu1MAINcomputed.mv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu1MAINcomputed.t, imu1MAINcomputed.mv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu1MAINcomputed.t, imu1MAINcomputed.mv(:,3), '*g')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Magnetic field of the center of mass of the multi-IMU system over time from IMU 1')
legend('mbx', 'mbx computed', 'mby', 'mby computed', 'mbz', 'mbz computed')
grid minor

[quat, euler] = attitude_computer(imu1MAINcomputed);

figure(4)
plot(navimu1MAINcomputed.t, quat(:,1), 'r', navimu1MAINcomputed.t, quat(:,2), 'c', navimu1MAINcomputed.t, quat(:,3), 'g', navimu1MAINcomputed.t, quat(:,4), 'k', ...
     navimu1MAINcomputed.t, navimu1MAINcomputed.qua(:,1), 'or', navimu1MAINcomputed.t, navimu1MAINcomputed.qua(:,2), 'oc', navimu1MAINcomputed.t, navimu1MAINcomputed.qua(:,3), 'og', navimu1MAINcomputed.t, navimu1MAINcomputed.qua(:,4), 'ok')
xlabel('Time [s]')
legend('q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions')
legend('location','southeast')

%% IMU NUMBER 2


imu2.ini_align = [0, 0, 0];
imu2.roll0 = -120;
imu2.pitch0 = 0;
imu2.yaw0 = 45;

imu2.DCMbv =  euler2dcm(deg2rad([imu2.roll0, imu2.pitch0, imu2.yaw0]));
imu2.DCMvb = imu2.DCMbv';
imu2.Rvb = imu2.DCMvb*[0, 0, L]';


[imu2] = VIMU_to_IMU(imuMAIN, imu2);


[imu2MAINcomputed] = IMU_to_VIMU(imu2);

figure(4)
plot(imuMAIN.t, imuMAIN.wv(:,1), 'b', imu2MAINcomputed.t, imu2MAINcomputed.wv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.wv(:,2), 'r', imu2MAINcomputed.t, imu2MAINcomputed.wv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.wv(:,3), 'g', imu2MAINcomputed.t, imu2MAINcomputed.wv(:,3), '*g')
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity of the center of mass of the multi-IMU system over time from IMU 2')
legend('wbx', 'wbx computed', 'wby', 'wby computed', 'wbz', 'wbz computed')
grid minor

figure(5)
plot(imuMAIN.t, imuMAIN.fv(:,1), 'b', imu2MAINcomputed.t, imu2MAINcomputed.fv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.fv(:,2), 'r', imu2MAINcomputed.t, imu2MAINcomputed.fv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.fv(:,3), 'g', imu2MAINcomputed.t, imu2MAINcomputed.fv(:,3), '*g')
xlabel('Time [s]')
ylabel('Acceleration [m/^2]')
title('Acceleration of the center of mass of the multi-IMU system over time from IMU 2')
legend('abx', 'abx computed', 'aby', 'aby computed', 'abz', 'abz computed')
grid minor

figure(6)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu2MAINcomputed.t, imu2MAINcomputed.mv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu2MAINcomputed.t, imu2MAINcomputed.mv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu2MAINcomputed.t, imu2MAINcomputed.mv(:,3), '*g')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Magnetic field of the center of mass of the multi-IMU system over time from IMU 2')
legend('mbx', 'mbx computed', 'mby', 'mby computed', 'mbz', 'mbz computed')
grid minor



%% IMU NUMBER 3


imu3.ini_align = [0, 0, 0];
imu3.roll0 = -120;
imu3.pitch0 = 0;
imu3.yaw0 = 180;

imu3.DCMbv =  euler2dcm(deg2rad([imu3.roll0, imu3.pitch0, imu3.yaw0]));
imu3.DCMvb = imu3.DCMbv';
imu3.Rvb = imu3.DCMvb*[0, 0, L]';


[imu3] = VIMU_to_IMU(imuMAIN, imu3);


[imu3MAINcomputed] = IMU_to_VIMU(imu3);

figure(7)
plot(imuMAIN.t, imuMAIN.wv(:,1), 'b', imu3MAINcomputed.t, imu3MAINcomputed.wv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.wv(:,2), 'r', imu3MAINcomputed.t, imu3MAINcomputed.wv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.wv(:,3), 'g', imu3MAINcomputed.t, imu3MAINcomputed.wv(:,3), '*g')
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity of the center of mass of the multi-IMU system over time from IMU 3')
legend('wbx', 'wbx computed', 'wby', 'wby computed', 'wbz', 'wbz computed')
grid minor


figure(8)
plot(imuMAIN.t, imuMAIN.fv(:,1), 'b', imu3MAINcomputed.t, imu3MAINcomputed.fv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.fv(:,2), 'r', imu3MAINcomputed.t, imu3MAINcomputed.fv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.fv(:,3), 'g', imu3MAINcomputed.t, imu3MAINcomputed.fv(:,3), '*g')
xlabel('Time [s]')
ylabel('Acceleration [m/^2]')
title('Acceleration of the center of mass of the multi-IMU system over time from IMU 3')
legend('abx', 'abx computed', 'aby', 'aby computed', 'abz', 'abz computed')
grid minor

figure(9)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu3MAINcomputed.t, imu3MAINcomputed.mv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu3MAINcomputed.t, imu3MAINcomputed.mv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu3MAINcomputed.t, imu3MAINcomputed.mv(:,3), '*g')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Magnetic field of the center of mass of the multi-IMU system over time from IMU 3')
legend('mbx', 'mbx computed', 'mby', 'mby computed', 'mbz', 'mbz computed')
grid minor

%% IMU NUMBER 4

imu4.ini_align = [0, 0, 0];
imu4.roll0 = -120;
imu4.pitch0 = 0;
imu4.yaw0 = -45;

imu4.DCMbv =  euler2dcm(deg2rad([imu4.roll0, imu4.pitch0, imu4.yaw0]));
imu4.DCMvb = imu4.DCMbv';
imu4.Rvb = imu4.DCMvb*[0, 0, L]';


[imu4] = VIMU_to_IMU(imuMAIN, imu4);


[imu4MAINcomputed] = IMU_to_VIMU(imu4);

figure(10)
plot(imuMAIN.t, imuMAIN.wv(:,1), 'b', imu4MAINcomputed.t, imu4MAINcomputed.wv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.wv(:,2), 'r', imu4MAINcomputed.t, imu4MAINcomputed.wv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.wv(:,3), 'g', imu4MAINcomputed.t, imu4MAINcomputed.wv(:,3), '*g')
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity of the center of mass of the multi-IMU system over time from IMU 4')
legend('wbx', 'wbx computed', 'wby', 'wby computed', 'wbz', 'wbz computed')
grid minor

figure(11)
plot(imuMAIN.t, imuMAIN.fv(:,1), 'b', imu4MAINcomputed.t, imu4MAINcomputed.fv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.fv(:,2), 'r', imu4MAINcomputed.t, imu4MAINcomputed.fv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.fv(:,3), 'g', imu4MAINcomputed.t, imu4MAINcomputed.fv(:,3), '*g')
xlabel('Time [s]')
ylabel('Acceleration [m/^2]')
title('Acceleration of the center of mass of the multi-IMU system over time from IMU 2')
legend('abx', 'abx computed', 'aby', 'aby computed', 'abz', 'abz computed')
grid minor

figure(12)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu4MAINcomputed.t, imu4MAINcomputed.mv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu4MAINcomputed.t, imu4MAINcomputed.mv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu4MAINcomputed.t, imu4MAINcomputed.mv(:,3), '*g')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Magnetic field of the center of mass of the multi-IMU system over time from IMU 4')
legend('mbx', 'mbx computed', 'mby', 'mby computed', 'mbz', 'mbz computed')
grid minor

