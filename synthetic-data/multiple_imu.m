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
%          7. Verify if the simulated motion for the IMUs coincides with
%             their theoretical value for constant z-axis rotation.
%          8. Apply the EKF to each of the resultling center of mass data
%             coming from each of the IMUs. Results: four different
%             attitudes coming from each IMU data.
% TO DO:
%          9. After step 5, combine/fuse the center of mass data obtained
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
imuMAIN.ini_align = [0, 0, 0];

% Generate synthetic motion by defining angular velocities
w = 4*pi/10;
imuMAIN.wv(:,1) = zeros(N, 1);
imuMAIN.wv(:,2) = zeros(N, 1);
imuMAIN.wv(:,3) = w*ones(N,1);

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

% convert the synthetic data of the body into data for the IMU1
[imu1] = VIMU_to_IMU(imuMAIN, imu1);

% convert the data of the IMU1 back to the center of mass
[imu1MAINcomputed] = IMU_to_VIMU(imu1);

% Initialize a priori data from the sensor
imu1MAINcomputed.ini_align = [0, 0, 0];
imu1MAINcomputed.ini_align_err = deg2rad([0.5, 0.5, 0.5]);
imu1MAINcomputed.gb_dyn = [0.0001, 0.0001, 0.0001];
imu1MAINcomputed.a_std = [0.05, 0.05, 0.05];
imu1MAINcomputed.g_std = [0.01, 0.01, 0.01];
imu1MAINcomputed.m_std = [0.005, 0.005, 0.005];

% Attitude EKF
imu1MAINcomputed.wb = imu1MAINcomputed.wv;
imu1MAINcomputed.fb = imu1MAINcomputed.fv;
imu1MAINcomputed.mb = imu1MAINcomputed.mv;

[navimu1MAINcomputed , kfimu1MAINcomputed] = imu_filter_magnetometer(imu1MAINcomputed);



% Plot angular velocity. Comparison between center of mass synthetic data
% and the one obtained from IMU1
figure(1)
plot(imuMAIN.t, imuMAIN.wv(:,1), 'b', imu1MAINcomputed.t, imu1MAINcomputed.wv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.wv(:,2), 'r', imu1MAINcomputed.t, imu1MAINcomputed.wv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.wv(:,3), 'g', imu1MAINcomputed.t, imu1MAINcomputed.wv(:,3), '*g')
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity of the center of mass of the multi-IMU system over time from IMU 1')
legend('wbx', 'wbx computed', 'wby', 'wby computed', 'wbz', 'wbz computed')
grid minor


% Plot acceleration. Comparison between center of mass synthetic data
% and the one obtained from IMU1
figure(2)
plot(imuMAIN.t, imuMAIN.fv(:,1), 'b', imu1MAINcomputed.t, imu1MAINcomputed.fv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.fv(:,2), 'r', imu1MAINcomputed.t, imu1MAINcomputed.fv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.fv(:,3), 'g', imu1MAINcomputed.t, imu1MAINcomputed.fv(:,3), '*g')
xlabel('Time [s]')
ylabel('Acceleration [m/^2]')
title('Acceleration of the center of mass of the multi-IMU system over time from IMU 1')
legend('abx', 'abx computed', 'aby', 'aby computed', 'abz', 'abz computed')
grid minor


% Plot local magnetic field. Comparison between center of mass synthetic data
% and the one obtained from IMU1
figure(3)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu1MAINcomputed.t, imu1MAINcomputed.mv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu1MAINcomputed.t, imu1MAINcomputed.mv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu1MAINcomputed.t, imu1MAINcomputed.mv(:,3), '*g')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Magnetic field of the center of mass of the multi-IMU system over time from IMU 1')
legend('mbx', 'mbx computed', 'mby', 'mby computed', 'mbz', 'mbz computed')
grid minor


% Compute attitude only using angular velocity data
[quat, euler] = attitude_computer(imu1MAINcomputed);
euler = rad2deg(euler);


% Plot comparison between true attitude vs EKF output
figure(4)
plot(navimu1MAINcomputed.t, quat(:,1), 'r', navimu1MAINcomputed.t, quat(:,2), 'c', navimu1MAINcomputed.t, quat(:,3), 'g', navimu1MAINcomputed.t, quat(:,4), 'k', ...
     navimu1MAINcomputed.t, navimu1MAINcomputed.qua(:,1), 'or', navimu1MAINcomputed.t, navimu1MAINcomputed.qua(:,2), 'oc', navimu1MAINcomputed.t, navimu1MAINcomputed.qua(:,3), 'og', navimu1MAINcomputed.t, navimu1MAINcomputed.qua(:,4), 'ok')
xlabel('Time [s]')
ylabel('quaternions')
legend('q1', 'q2,', 'q3', 'q4', 'q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions')
legend('location','southeast')


figure(5)
plot(navimu1MAINcomputed.t, euler(:,1), 'r', navimu1MAINcomputed.t, euler(:,2), 'c', navimu1MAINcomputed.t, euler(:,3), 'g', ...
     navimu1MAINcomputed.t, navimu1MAINcomputed.roll, 'or', navimu1MAINcomputed.t, navimu1MAINcomputed.pitch, 'oc', navimu1MAINcomputed.t, navimu1MAINcomputed.yaw, 'og')
xlabel('Time [s]')
ylabel('Euler angles [deg]')
legend('roll', 'pitch,', 'yaw', 'roll Kalman','pitch Kalman','yaw Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions')
legend('location','southeast')


% Plot angular velocity. Center of mass vs IMU1
figure(6)
plot(imuMAIN.t, imuMAIN.wv(:,1), 'b', imu1.t, imu1.wb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.wv(:,2), 'r', imu1.t, imu1.wb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.wv(:,3), 'g', imu1.t, imu1.wb(:,3), 'og')
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity center of mass vs imu1')
legend('wx CM',  'wx imu1', 'wy CM', 'wy imu1', 'wz CM', 'wz imu1')
grid minor


% Plot acceleration. Comparison between center of mass synthetic data
% and the one obtained from IMU1
figure(7)
plot(imuMAIN.t, imuMAIN.fv(:,1), 'b', imu1.t, imu1.fb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.fv(:,2), 'r', imu1.t, imu1.fb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.fv(:,3), 'g', imu1.t, imu1.fb(:,3), 'og')
xlabel('Time [s]')
ylabel('Acceleration [m/^2]')
title('Acceleration center of mass vs imu1')
legend('ax CM',  'ax imu1', 'ay CM', 'ay imu1', 'az CM', 'az imu1')
grid minor


% Plot local magnetic field. Comparison between center of mass synthetic data
% and the one obtained from IMU1
figure(8)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu1.t, imu1.mb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu1.t, imu1.mb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu1.t, imu1.mb(:,3), 'og')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Local magnetic field center of mass vs imu1')
legend('mx CM',  'mx imu1', 'my CM', 'my imu1', 'mz CM', 'mz imu1')
grid minor


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

% convert the data of the IMU2 back to the center of mass
[imu2MAINcomputed] = IMU_to_VIMU(imu2);

% Initialize a priori data from the sensor
imu2MAINcomputed.ini_align = [0, 0, 0];
imu2MAINcomputed.ini_align_err = deg2rad([0.5, 0.5, 0.5]);
imu2MAINcomputed.gb_dyn = [0.0001, 0.0001, 0.0001];
imu2MAINcomputed.a_std = [0.05, 0.05, 0.05];
imu2MAINcomputed.g_std = [0.01, 0.01, 0.01];
imu2MAINcomputed.m_std = [0.005, 0.005, 0.005];

% Attitude EKF
imu2MAINcomputed.wb = imu2MAINcomputed.wv;
imu2MAINcomputed.fb = imu2MAINcomputed.fv;
imu2MAINcomputed.mb = imu2MAINcomputed.mv;

[navimu2MAINcomputed , kfimu2MAINcomputed] = imu_filter_magnetometer(imu2MAINcomputed);



% Plot angular velocity. Comparison between center of mass synthetic data
% and the one obtained from IMU2
figure(9)
plot(imuMAIN.t, imuMAIN.wv(:,1), 'b', imu2MAINcomputed.t, imu2MAINcomputed.wv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.wv(:,2), 'r', imu2MAINcomputed.t, imu2MAINcomputed.wv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.wv(:,3), 'g', imu2MAINcomputed.t, imu2MAINcomputed.wv(:,3), '*g')
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity of the center of mass of the multi-IMU system over time from IMU 2')
legend('wbx', 'wbx computed', 'wby', 'wby computed', 'wbz', 'wbz computed')
grid minor


% Plot acceleration. Comparison between center of mass synthetic data
% and the one obtained from IMU2
figure(10)
plot(imuMAIN.t, imuMAIN.fv(:,1), 'b', imu2MAINcomputed.t, imu2MAINcomputed.fv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.fv(:,2), 'r', imu2MAINcomputed.t, imu2MAINcomputed.fv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.fv(:,3), 'g', imu2MAINcomputed.t, imu2MAINcomputed.fv(:,3), '*g')
xlabel('Time [s]')
ylabel('Acceleration [m/s^2]')
title('Acceleration of the center of mass of the multi-IMU system over time from IMU 2')
legend('abx', 'abx computed', 'aby', 'aby computed', 'abz', 'abz computed')
grid minor


% Plot local magnetic field. Comparison between center of mass synthetic data
% and the one obtained from IMU2
figure(11)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu2MAINcomputed.t, imu2MAINcomputed.mv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu2MAINcomputed.t, imu2MAINcomputed.mv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu2MAINcomputed.t, imu2MAINcomputed.mv(:,3), '*g')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Magnetic field of the center of mass of the multi-IMU system over time from IMU 2')
legend('mbx', 'mbx computed', 'mby', 'mby computed', 'mbz', 'mbz computed')
grid minor


% Compute attitude only using angular velocity data
[quat, euler] = attitude_computer(imu2MAINcomputed);
euler = rad2deg(euler);


% Plot comparison between true attitude vs EKF output
figure(12)
plot(navimu2MAINcomputed.t, quat(:,1), 'r', navimu2MAINcomputed.t, quat(:,2), 'c', navimu2MAINcomputed.t, quat(:,3), 'g', navimu2MAINcomputed.t, quat(:,4), 'k', ...
     navimu2MAINcomputed.t, navimu2MAINcomputed.qua(:,1), 'or', navimu2MAINcomputed.t, navimu2MAINcomputed.qua(:,2), 'oc', navimu2MAINcomputed.t, navimu2MAINcomputed.qua(:,3), 'og', navimu2MAINcomputed.t, navimu2MAINcomputed.qua(:,4), 'ok')
xlabel('Time [s]')
legend('q1', 'q2,', 'q3', 'q4', 'q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions')
legend('location','southeast')


figure(13)
plot(navimu2MAINcomputed.t, euler(:,1), 'r', navimu2MAINcomputed.t, euler(:,2), 'c', navimu2MAINcomputed.t, euler(:,3), 'g', ...
     navimu2MAINcomputed.t, navimu2MAINcomputed.roll, 'or', navimu2MAINcomputed.t, navimu2MAINcomputed.pitch, 'oc', navimu2MAINcomputed.t, navimu2MAINcomputed.yaw, 'og')
xlabel('Time [s]')
legend('roll', 'pitch,', 'yaw', 'roll Kalman','pitch Kalman','yaw Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions')
legend('location','southeast')


% Plot angular velocity. Center of mass vs IMU2
figure(14)
plot(imuMAIN.t, imuMAIN.wv(:,1), 'b', imu1.t, imu2.wb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.wv(:,2), 'r', imu1.t, imu2.wb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.wv(:,3), 'g', imu1.t, imu2.wb(:,3), 'og')
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity center of mass vs imu2')
legend('wx CM',  'wx imu2', 'wy CM', 'wy imu2', 'wz CM', 'wz imu2')
grid minor


% Plot acceleration. Comparison between center of mass synthetic data
% and the one obtained from IMU2
figure(15)
plot(imuMAIN.t, imuMAIN.fv(:,1), 'b', imu1.t, imu2.fb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.fv(:,2), 'r', imu1.t, imu2.fb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.fv(:,3), 'g', imu1.t, imu2.fb(:,3), 'og')
xlabel('Time [s]')
ylabel('Acceleration [m/^2]')
title('Acceleration center of mass vs imu2')
legend('ax CM',  'ax imu2', 'ay CM', 'ay imu2', 'az CM', 'az imu2')
grid minor


% Plot local magnetic field. Comparison between center of mass synthetic data
% and the one obtained from IMU2
figure(16)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu2.t, imu2.mb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu2.t, imu2.mb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu2.t, imu2.mb(:,3), 'og')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Local magnetic field center of mass vs imu2')
legend('mx CM',  'mx imu2', 'my CM', 'my imu2', 'mz CM', 'mz imu2')
grid minor



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

% convert the data of the IMU2 back to the center of mass
[imu3MAINcomputed] = IMU_to_VIMU(imu3);

% Initialize a priori data from the sensor
imu3MAINcomputed.ini_align = [0, 0, 0];
imu3MAINcomputed.ini_align_err = deg2rad([0.5, 0.5, 0.5]);
imu3MAINcomputed.gb_dyn = [0.0001, 0.0001, 0.0001];
imu3MAINcomputed.a_std = [0.05, 0.05, 0.05];
imu3MAINcomputed.g_std = [0.01, 0.01, 0.01];
imu3MAINcomputed.m_std = [0.005, 0.005, 0.005];

% Attitude EKF
imu3MAINcomputed.wb = imu3MAINcomputed.wv;
imu3MAINcomputed.fb = imu3MAINcomputed.fv;
imu3MAINcomputed.mb = imu3MAINcomputed.mv;

[navimu3MAINcomputed , kfimu3MAINcomputed] = imu_filter_magnetometer(imu3MAINcomputed);



% Plot angular velocity. Comparison between center of mass synthetic data
% and the one obtained from IMU3
figure(17)
plot(imuMAIN.t, imuMAIN.wv(:,1), 'b', imu3MAINcomputed.t, imu3MAINcomputed.wv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.wv(:,2), 'r', imu3MAINcomputed.t, imu3MAINcomputed.wv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.wv(:,3), 'g', imu3MAINcomputed.t, imu3MAINcomputed.wv(:,3), '*g')
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity of the center of mass of the multi-IMU system over time from IMU 3')
legend('wbx', 'wbx computed', 'wby', 'wby computed', 'wbz', 'wbz computed')
grid minor


% Plot acceleration. Comparison between center of mass synthetic data
% and the one obtained from IMU3
figure(18)
plot(imuMAIN.t, imuMAIN.fv(:,1), 'b', imu3MAINcomputed.t, imu3MAINcomputed.fv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.fv(:,2), 'r', imu3MAINcomputed.t, imu3MAINcomputed.fv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.fv(:,3), 'g', imu3MAINcomputed.t, imu3MAINcomputed.fv(:,3), '*g')
xlabel('Time [s]')
ylabel('Acceleration [m/s^2]')
title('Acceleration of the center of mass of the multi-IMU system over time from IMU 3')
legend('abx', 'abx computed', 'aby', 'aby computed', 'abz', 'abz computed')
grid minor


% Plot local magnetic field. Comparison between center of mass synthetic data
% and the one obtained from IMU3
figure(19)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu3MAINcomputed.t, imu3MAINcomputed.mv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu3MAINcomputed.t, imu3MAINcomputed.mv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu3MAINcomputed.t, imu3MAINcomputed.mv(:,3), '*g')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Magnetic field of the center of mass of the multi-IMU system over time from IMU 3')
legend('mbx', 'mbx computed', 'mby', 'mby computed', 'mbz', 'mbz computed')
grid minor


% Compute attitude only using angular velocity data
[quat, euler] = attitude_computer(imu3MAINcomputed);
euler = rad2deg(euler);


% Plot comparison between true attitude vs EKF output
figure(20)
plot(navimu3MAINcomputed.t, quat(:,1), 'r', navimu3MAINcomputed.t, quat(:,2), 'c', navimu3MAINcomputed.t, quat(:,3), 'g', navimu3MAINcomputed.t, quat(:,4), 'k', ...
     navimu3MAINcomputed.t, navimu3MAINcomputed.qua(:,1), 'or', navimu3MAINcomputed.t, navimu3MAINcomputed.qua(:,2), 'oc', navimu3MAINcomputed.t, navimu3MAINcomputed.qua(:,3), 'og', navimu3MAINcomputed.t, navimu3MAINcomputed.qua(:,4), 'ok')
xlabel('Time [s]')
legend('q1', 'q2,', 'q3', 'q4', 'q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions')
legend('location','southeast')


figure(21)
plot(navimu3MAINcomputed.t, euler(:,1), 'r', navimu3MAINcomputed.t, euler(:,2), 'c', navimu3MAINcomputed.t, euler(:,3), 'g', ...
     navimu3MAINcomputed.t, navimu3MAINcomputed.roll, 'or', navimu3MAINcomputed.t, navimu3MAINcomputed.pitch, 'oc', navimu3MAINcomputed.t, navimu3MAINcomputed.yaw, 'og')
xlabel('Time [s]')
legend('roll', 'pitch,', 'yaw', 'roll Kalman','pitch Kalman','yaw Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions')
legend('location','southeast')


% Plot angular velocity. Center of mass vs IMU3
figure(22)
plot(imuMAIN.t, imuMAIN.wv(:,1), 'b', imu3.t, imu3.wb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.wv(:,2), 'r', imu3.t, imu3.wb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.wv(:,3), 'g', imu3.t, imu3.wb(:,3), 'og')
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity center of mass vs imu3')
legend('wx CM',  'wx imu3', 'wy CM', 'wy imu3', 'wz CM', 'wz imu3')
grid minor


% Plot acceleration. Comparison between center of mass synthetic data
% and the one obtained from IMU3
figure(23)
plot(imuMAIN.t, imuMAIN.fv(:,1), 'b', imu3.t, imu3.fb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.fv(:,2), 'r', imu3.t, imu3.fb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.fv(:,3), 'g', imu3.t, imu3.fb(:,3), 'og')
xlabel('Time [s]')
ylabel('Acceleration [m/^2]')
title('Acceleration center of mass vs imu2')
legend('ax CM',  'ax imu3', 'ay CM', 'ay imu3', 'az CM', 'az imu3')
grid minor


% Plot local magnetic field. Comparison between center of mass synthetic data
% and the one obtained from IMU3
figure(24)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu3.t, imu3.mb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu3.t, imu3.mb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu3.t, imu3.mb(:,3), 'og')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Local magnetic field center of mass vs imu3')
legend('mx CM',  'mx imu3', 'my CM', 'my imu3', 'mz CM', 'mz imu3')



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

% convert the data of the IMU2 back to the center of mass
[imu4MAINcomputed] = IMU_to_VIMU(imu4);

% Initialize a priori data from the sensor
imu4MAINcomputed.ini_align = [0, 0, 0];
imu4MAINcomputed.ini_align_err = deg2rad([0.5, 0.5, 0.5]);
imu4MAINcomputed.gb_dyn = [0.0001, 0.0001, 0.0001];
imu4MAINcomputed.a_std = [0.05, 0.05, 0.05];
imu4MAINcomputed.g_std = [0.01, 0.01, 0.01];
imu4MAINcomputed.m_std = [0.005, 0.005, 0.005];

% Attitude EKF
imu4MAINcomputed.wb = imu4MAINcomputed.wv;
imu4MAINcomputed.fb = imu4MAINcomputed.fv;
imu4MAINcomputed.mb = imu4MAINcomputed.mv;

[navimu4MAINcomputed , kfimu4MAINcomputed] = imu_filter_magnetometer(imu4MAINcomputed);



% Plot angular velocity. Comparison between center of mass synthetic data
% and the one obtained from IMU3
figure(25)
plot(imuMAIN.t, imuMAIN.wv(:,1), 'b', imu4MAINcomputed.t, imu4MAINcomputed.wv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.wv(:,2), 'r', imu4MAINcomputed.t, imu4MAINcomputed.wv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.wv(:,3), 'g', imu4MAINcomputed.t, imu4MAINcomputed.wv(:,3), '*g')
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity of the center of mass of the multi-IMU system over time from IMU 4')
legend('wbx', 'wbx computed', 'wby', 'wby computed', 'wbz', 'wbz computed')
grid minor


% Plot acceleration. Comparison between center of mass synthetic data
% and the one obtained from IMU4
figure(26)
plot(imuMAIN.t, imuMAIN.fv(:,1), 'b', imu4MAINcomputed.t, imu4MAINcomputed.fv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.fv(:,2), 'r', imu4MAINcomputed.t, imu4MAINcomputed.fv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.fv(:,3), 'g', imu4MAINcomputed.t, imu4MAINcomputed.fv(:,3), '*g')
xlabel('Time [s]')
ylabel('Acceleration [m/s^2]')
title('Acceleration of the center of mass of the multi-IMU system over time from IMU 4')
legend('abx', 'abx computed', 'aby', 'aby computed', 'abz', 'abz computed')
grid minor


% Plot local magnetic field. Comparison between center of mass synthetic data
% and the one obtained from IMU3
figure(27)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu4MAINcomputed.t, imu4MAINcomputed.mv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu4MAINcomputed.t, imu4MAINcomputed.mv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu4MAINcomputed.t, imu4MAINcomputed.mv(:,3), '*g')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Magnetic field of the center of mass of the multi-IMU system over time from IMU 4')
legend('mbx', 'mbx computed', 'mby', 'mby computed', 'mbz', 'mbz computed')
grid minor


% Compute attitude only using angular velocity data
[quat, euler] = attitude_computer(imu3MAINcomputed);
euler = rad2deg(euler);


% Plot comparison between true attitude vs EKF output
figure(28)
plot(navimu4MAINcomputed.t, quat(:,1), 'r', navimu4MAINcomputed.t, quat(:,2), 'c', navimu4MAINcomputed.t, quat(:,3), 'g', navimu4MAINcomputed.t, quat(:,4), 'k', ...
     navimu4MAINcomputed.t, navimu4MAINcomputed.qua(:,1), 'or', navimu4MAINcomputed.t, navimu4MAINcomputed.qua(:,2), 'oc', navimu4MAINcomputed.t, navimu4MAINcomputed.qua(:,3), 'og', navimu4MAINcomputed.t, navimu4MAINcomputed.qua(:,4), 'ok')
xlabel('Time [s]')
legend('q1', 'q2,', 'q3', 'q4', 'q1 Kalman','q2 Kalman','q3 Kalman', 'q4 Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions')
legend('location','southeast')


figure(29)
plot(navimu4MAINcomputed.t, euler(:,1), 'r', navimu4MAINcomputed.t, euler(:,2), 'c', navimu4MAINcomputed.t, euler(:,3), 'g', ...
     navimu4MAINcomputed.t, navimu4MAINcomputed.roll, 'or', navimu4MAINcomputed.t, navimu4MAINcomputed.pitch, 'oc', navimu4MAINcomputed.t, navimu4MAINcomputed.yaw, 'og')
xlabel('Time [s]')
legend('roll', 'pitch,', 'yaw', 'roll Kalman','pitch Kalman','yaw Kalman')
grid minor
title('Attitude computer vs Kalman filter. Quaternions')
legend('location','southeast')


% Plot angular velocity. Center of mass vs IMU4
figure(30)
plot(imuMAIN.t, imuMAIN.wv(:,1), 'b', imu4.t, imu4.wb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.wv(:,2), 'r', imu4.t, imu4.wb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.wv(:,3), 'g', imu4.t, imu4.wb(:,3), 'og')
xlabel('Time [s]')
ylabel('Angular velocity [rad/s]')
title('Angular velocity center of mass vs imu4')
legend('wx CM',  'wx imu4', 'wy CM', 'wy imu4', 'wz CM', 'wz imu4')
grid minor


% Plot acceleration. Comparison between center of mass synthetic data
% and the one obtained from IMU4
figure(31)
plot(imuMAIN.t, imuMAIN.fv(:,1), 'b', imu4.t, imu4.fb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.fv(:,2), 'r', imu4.t, imu4.fb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.fv(:,3), 'g', imu4.t, imu4.fb(:,3), 'og')
xlabel('Time [s]')
ylabel('Acceleration [m/^2]')
title('Acceleration center of mass vs imu4')
legend('ax CM',  'ax imu4', 'ay CM', 'ay imu4', 'az CM', 'az imu4')
grid minor


% Plot local magnetic field. Comparison between center of mass synthetic data
% and the one obtained from IMU4
figure(32)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu4.t, imu4.mb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu4.t, imu4.mb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu4.t, imu4.mb(:,3), 'og')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Local magnetic field center of mass vs imu4')
legend('mx CM',  'mx imu4', 'my CM', 'my imu4', 'mz CM', 'mz imu4')


%% VERIFICATION
% In a z-axis center of mass constant rotation, 
%       1. IMU 1 should have same ang velocity, acceleration and magnetic
%       field values that MAIN
%       2. IMUS 2,3,4 will experiment these values for the acceleration in their
%       local body frame:
%          fb = [g*cos(30) + an*sin(30), 0, g*sin(30) - an*cos(30)]

% acceleration of gravity employed in IMU_simulator
g = 9.8;

% radius for centripetal acceleration
R = L*cosd(30);

% centripetal acceleration
an = w^2*R;

fprintf('CHECK RESULTS FOR IMU 1 \n\n')

fprintf('THEORETICAL RESULTS FOR IMU 1 \n')
fprintf('fbx should be %f m/s^2 \n', imuMAIN.fv(1,1))
fprintf('fby should be %f m/s^2 \n', imuMAIN.fv(1,2))
fprintf('fbz should be %f m/s^2 \n\n', imuMAIN.fv(1,3))

fprintf('ACTUAL RESULTS FOR IMU 1 \n')
fprintf('fbx is %f m/s^2 \n', imu1.fb(1,1))
fprintf('fby is %f m/s^2 \n', imu1.fb(1,2))
fprintf('fbz is %f m/s^2 \n\n\n', imu1.fb(1,3))


fprintf('CHECK RESULTS FOR IMU 2, 3, 4 \n\n')

fprintf('THEORETICAL RESULTS FOR IMUS 2,3,4 \n')
fprintf('fbx should be %f m/s^2 \n', g*cosd(30) + an*sind(30))
fprintf('fby should be %f m/s^2 \n', 0)
fprintf('fbz should be %f m/s^2 \n\n', g*sind(30) - an*cosd(30))

fprintf('ACTUAL RESULTS FOR IMU 2 \n')
fprintf('fbx is %f m/s^2 \n', imu2.fb(1,1))
fprintf('fby is %f m/s^2 \n', imu2.fb(1,2))
fprintf('fbz is %f m/s^2 \n\n', imu2.fb(1,3))

fprintf('ACTUAL RESULTS FOR IMU 3 \n')
fprintf('fbx is %f m/s^2 \n', imu3.fb(1,1))
fprintf('fby is %f m/s^2 \n', imu3.fb(1,2))
fprintf('fbz is %f m/s^2 \n\n', imu3.fb(1,3))

fprintf('ACTUAL RESULTS FOR IMU 4 \n')
fprintf('fbx is %f m/s^2 \n', imu4.fb(1,1))
fprintf('fby is %f m/s^2 \n', imu4.fb(1,2))
fprintf('fbz is %f m/s^2 \n\n', imu4.fb(1,3))
