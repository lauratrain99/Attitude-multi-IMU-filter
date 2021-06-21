
t = 0:1/100:10;

%% SIMULATE MOTION FOR THE BODY (CENTER OF MASS)

% Initial considerations:
% B, body reference frame of the each of the IMUs
% V, body reference frame of the center of mass of the multi-IMU body
% DCMvb is the DCM from the ith-imu body reference frame to the 
% center of mass of body. It is fixed and known a priori
% imuMAIN, the synthetic motion of the center of mass

imuMAIN.t = t;
N = length(imuMAIN.t);
imuMAIN.ini_align = [0, 0, 0];

% Generate synthetic motion by defining angular velocities
w = 0;
imuMAIN.wv(:,1) = zeros(N, 1);
imuMAIN.wv(:,2) = zeros(N, 1);
imuMAIN.wv(:,3) = w*ones(N,1);

% Generate synthetic motion for the acceleration and magnetic field
[imuMAIN] = IMU_simulator(imuMAIN);


%% GENERATE DATA FOR IMU NUMBER 1
% imu1, imu located at a point different from the center of mass
% imu1MAINcomputed, the data for the center of mass obtained 
% from the IMU1

% length from the center of mass to the IMU1
L = 0.065;

% relative orientation of the IMU1 with respect to the center of mass 
% of the body
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

% convert the data of the IMU1 back to the center of mass
[imu1MAINcomputed] = IMU_to_VIMU(imu1);


%% GENERATE DATA FOR IMU NUMBER 2
% imu2, imu located at a point different from the center of mass
% imu2MAINcomputed, the data for the center of mass obtained 
% from the IMU2

% length from the center of mass to the IMU2
L = 0.065;

% relative orientation of the IMU1 with respect to the center of mass
% of the body
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


%% GENERATE DATA FOR IMU NUMBER 3
% imu3, imu located at a point different from the center of mass
% imu3MAINcomputed, the data for the center of mass obtained 
% from the IMU3

% length from the center of mass to the IMU3
L = 0.065;

% relative orientation of the IMU3 with respect to the center of mass 
% of the body
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


%% GENERATE DATA FOR IMU NUMBER 4
% imu4, imu located at a point different from the center of mass
% imu4MAINcomputed, the data for the center of mass obtained
% from the IMU4

% length from the center of mass to the IMU4
L = 0.065;

% relative orientation of the IMU4 with respect to the center of mass
% of the body
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


%% VERIFICATION
% In a z-axis center of mass constant rotation, 
% 1. IMU 1 should have same ang velocity, acceleration and magnetic
% field values that MAIN
% 2. IMUS 2,3,4 will experiment these values for the acceleration 
% in their local body frame:
%         fb = [g*cos(30) + an*sin(30), 0, g*sin(30) - an*cos(30)]

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
fprintf('fbx should be %f m/s^2 \n', -g*cosd(30) + an*sind(30))
fprintf('fby should be %f m/s^2 \n', 0)
fprintf('fbz should be %f m/s^2 \n\n', -g*sind(30) - an*cosd(30))

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
