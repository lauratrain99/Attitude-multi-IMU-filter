%% MULTIPLE IMU KINEMATIC VERIFICATION
% Author: Laura Train
% Date of the last update Jan 22 2021
%
% The goal of this script is to verify the correct functionality of a
% multi-IMU configuration (four sensors) to estimate attitude. The sensors
% are placed at locations different from the center of mass of the body.
% The test consists of the simulation of a z-axis constant rotation for 
% which the motion of each IMU at their local body reference frame is known
% and easy to check out.
%
% Checklist:
% DONE:
%          1. Generate synthetic data for the center of mass of a body.
%             Input values are angular velocities, accelerations and local
%             magnetic field values are computed from the w's.
%    
%          3. Number of IMUs: 4. Define the orientation of each of them 
%             with respect to the center of mass by the variables roll0, pitch0, yaw0.
%
%          4. Convert the synthetic gyro, accel and magn data of the center
%             of mass into gyro, accel and magn data of the body reference.
%
%          5. Transform gyro, accel and magn data of each of the IMUs into
%             the data of the center of mass of the body.
%
%          6. Plot the data from the initial synthetic data to see if it
%             coincides with the transformed data.
%
%          7. Verify if the simulated motion for the IMUs coincides with
%             their theoretical value for constant z-axis rotation.



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


% Plot angular velocity. Center of mass vs IMU1
figure(4)
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
figure(5)
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
figure(6)
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




% Plot angular velocity. Comparison between center of mass synthetic data
% and the one obtained from IMU2
figure(7)
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
figure(8)
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
figure(9)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu2MAINcomputed.t, imu2MAINcomputed.mv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu2MAINcomputed.t, imu2MAINcomputed.mv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu2MAINcomputed.t, imu2MAINcomputed.mv(:,3), '*g')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Magnetic field of the center of mass of the multi-IMU system over time from IMU 2')
legend('mbx', 'mbx computed', 'mby', 'mby computed', 'mbz', 'mbz computed')
grid minor



% Plot angular velocity. Center of mass vs IMU2
figure(10)
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
figure(11)
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
figure(12)
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




% Plot angular velocity. Comparison between center of mass synthetic data
% and the one obtained from IMU3
figure(13)
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
figure(14)
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
figure(15)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu3MAINcomputed.t, imu3MAINcomputed.mv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu3MAINcomputed.t, imu3MAINcomputed.mv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu3MAINcomputed.t, imu3MAINcomputed.mv(:,3), '*g')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Magnetic field of the center of mass of the multi-IMU system over time from IMU 3')
legend('mbx', 'mbx computed', 'mby', 'mby computed', 'mbz', 'mbz computed')
grid minor



% Plot angular velocity. Center of mass vs IMU3
figure(16)
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
figure(17)
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
figure(18)
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



% Plot angular velocity. Comparison between center of mass synthetic data
% and the one obtained from IMU3
figure(19)
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
figure(20)
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
figure(21)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu4MAINcomputed.t, imu4MAINcomputed.mv(:,1), '*b', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu4MAINcomputed.t, imu4MAINcomputed.mv(:,2), '*r', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu4MAINcomputed.t, imu4MAINcomputed.mv(:,3), '*g')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Magnetic field of the center of mass of the multi-IMU system over time from IMU 4')
legend('mbx', 'mbx computed', 'mby', 'mby computed', 'mbz', 'mbz computed')
grid minor


% Plot angular velocity. Center of mass vs IMU4
figure(22)
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
figure(23)
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
figure(24)
plot(imuMAIN.t, imuMAIN.mv(:,1), 'b', imu4.t, imu4.mb(:,1), 'ob', ...
     imuMAIN.t, imuMAIN.mv(:,2), 'r', imu4.t, imu4.mb(:,2), 'or', ...
     imuMAIN.t, imuMAIN.mv(:,3), 'g', imu4.t, imu4.mb(:,3), 'og')
xlabel('Time [s]')
ylabel('Magnetic field [Gauss]')
title('Local magnetic field center of mass vs imu4')
legend('mx CM',  'mx imu4', 'my CM', 'my imu4', 'mz CM', 'mz imu4')
grid minor

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
