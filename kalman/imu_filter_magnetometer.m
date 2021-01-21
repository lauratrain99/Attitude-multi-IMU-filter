function [nav , kf] = imu_filter_magnetometer(imu)
%
% imu_filter_magnetometer filters IMU measurements by using an Extended Kalman filter.
% The IMU measurements include accelerometer, gyro and magnetometer data.
% INPUT
%           imu, IMU data structure.
%             t: Nx1 time vector (seconds).
%            fb: Nx3 accelerations vector in body frame XYZ (m/s^2)
%            wb: Nx3 turn rates vector in body frame XYZ (radians/s)
%            mb: Nx3 magnetic field vector in NED frame (Gauss)
%         g_std: Nx3 gyros standard deviations (radians/s)
%         a_std: Nx3 accrs standard deviations (m/s^2)
%         m_std: Nx3 magns standard deviations (Gauss)
%        gb_dyn: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%     ini_align: 1x3 initial attitude at t(1).
% ini_align_err: 1x3 initial attitude errors at t(1).
%
%
% OUTPUT
%          nav, INS navigation estimates data structure.
%            t: Nx1 INS time vector (seconds).
%         roll: Nx1 roll (radians).
%        pitch: Nx1 pitch (radians).
%          yaw: Nx1 yaw (radians).
%          qua: Mx4 corrected quaternions.
%        DCMbn: Mx9 corrected DCM matrix from body-to-nav frame.
%      deltaxi: Mx6 Kalman filter a priori error states.
%      deltaxp: Mx6 Kalman filter a posteriori error states.
%          Phi: Mx36 Kalman filter transition-state matrices, one matrix per
%               row ordered by columns.
%           Pi: Mx36 Kalman filter a priori covariance matrices, one matrix
%               per row ordered by columns.
%           Pp: Mx36 Kalman filter a posteriori covariance matrices, one
%               matrix per row ordered by columns.
%            K: Mx36 Kalman filter gain matrix.
%            S: Mx36 Kalman filter innovation matrix.
%           ob: Mx1 Evolution of number of observable states.
%  deltay_prop: Mx3 Evolution of the propagated measurement error
%       deltar: Mx3 Evolution of the error residual
%       deltay: Mx3 Evolution of the estimated measurement error
%           wb: Mx3 Evolution of the corrected angular velocity

%% PREALLOCATION AND INITIALIZATION

% Constant matrices
I = eye(3);
O = zeros(3);


% Length of time vector

LI = length(imu.t);
%LI = 10000;


% Preallocation of attitude vectors
nav.roll  = zeros (LI, 1);
nav.pitch = zeros (LI, 1);
nav.yaw   = zeros (LI, 1);
nav.qua   = zeros(LI, 4);
nav.DCMbn = zeros(LI, 9);


% Initial attitude at time = 1
roll  = imu.ini_align(1);
pitch = imu.ini_align(2);
yaw   = imu.ini_align(3);
euler = [roll, pitch, yaw];

nav.roll(1)  = imu.ini_align(1);
nav.pitch(1) = imu.ini_align(2);
nav.yaw(1)   = imu.ini_align(3);

qua = euler2qua(euler);
DCMnb = euler2dcm(euler);
DCMbn = DCMnb';

nav.qua(1,:) = qua;
nav.DCMbn(1,:) = reshape(DCMbn, 1, 9);


% Initial dynamic bias
gb_dyn = imu.gb_dyn';


% Preallocation of Kalman filter matrices for later performance analysis
nav.t = zeros(LI,1);             % Discrete time for the results
nav.deltaxi = zeros(LI, 6);      % Evolution of Kalman filter a priori states, xi
nav.deltaxp = zeros(LI, 6);      % Evolution of Kalman filter a posteriori states, xp
nav.Phi  = zeros(LI, 36);        % Transition-state matrices, Phi
nav.Pi = zeros(LI, 36);          % A priori covariance matrices, Pi
nav.Pp = zeros(LI, 36);          % A posteriori covariance matrices, Pp
nav.K  = zeros(LI, 36);          % Kalman gain matrices, K
nav.S  = zeros(LI, 36);           % Innovation matrices, S
nav.ob = zeros(LI, 1);           % Number of observable states at each acceleromter data
nav.deltar = zeros(LI,6);        % Error residual
nav.wb = zeros(LI, 3);           % Corrected angular velocity

% Prior estimates for initial update
kf.deltaxi = [zeros(1,3), imu.gb_dyn]';            % Error vector state
kf.Pi = diag([imu.ini_align_err, imu.gb_dyn].^2);

ge = 9.81;
mN = 0.22;
mD = 0.17;
g_n = [0; 0; ge];
m_n = [mN; 0; mD];
ab = imu.fb(1,:)';
mb = imu.mb(1,:)';
kf.deltar = [-DCMbn*ab - g_n; DCMbn*mb - m_n];



% Measurement matrix, constant value over time
kf.H = [skewm(g_n) O; skewm(m_n) O];

% Correction covariance matrix, constant value over time
kf.R = diag([imu.a_std, imu.m_std]).^2;
%kf.R = diag([0.0001, 0.0001, 0.0001]).^2;

% Propagate prior estimates to get xp(1) and Pp(1)
kf = kf_update( kf );



% Initial matrices for Kalman filter performance analysis
nav.deltaxi(1,:) = kf.deltaxi;
nav.Pi = reshape(kf.Pi, 1, 36);

nav.deltaxp(1,:) = kf.deltaxp;
nav.Pp(1,:) = reshape(kf.Pp, 1, 36);
nav.deltar(1,:) = kf.deltar;
nav.wb(1,:) = imu.wb(1,:)' - gb_dyn;


% Prediction covariance matrix, constant value over time
kf.Q  = diag([imu.g_std, imu.gb_dyn].^2);
%kf.Q  = diag([1, 1, 1, 1, 1, 1].^2);


for i = 2:LI

      % IMU sampling interval
      dt = imu.t(i) - imu.t(i-1);
     
      % Correction for angular velocity with bias inestability
      wb_corrected = imu.wb(i,:)' - gb_dyn;
     
      % Attitude update 3-2-1 body sequence
      [qua, DCMbn, euler] = my_quat_update(wb_corrected, qua, dt);

      % KALMAN FILTER      
          % PREDICTION         
          kf.F = [-skewm(wb_corrected) -I; O O];
          %kf.J = [-DCMbn -I O; O O I];
          kf.G = [-I O; O I];
          
          % CORRECTION
          ab = imu.fb(i,:)';
          mb = imu.mb(i,:)';
          kf.deltar = [-DCMbn*ab - g_n; DCMbn*mb - m_n];
          
          % Execution of the Extended Kalman filter
          kf.deltaxp = zeros(length(kf.deltaxp),1);           % states 1:3 are forced to be zero (error-state approach)
          kf = kalman(kf, dt);
         
          % OBSERVABILITY
          % Number the observable states
          ob = rank(obsv(kf.F, kf.H));
         
          % ADD THE ERROR TO THE STATE
          % Quaternion corrections
          % Crassidis. Eq. 7.34 and A.174a.
          antm = [0 qua(3) -qua(2); -qua(3) 0 qua(1); qua(2) -qua(1) 0];
          qua = qua + 0.5 .* [qua(4)*eye(3) + antm; -1.*[qua(1) qua(2) qua(3)]] * kf.deltaxp(1:3);
          qua = qua / norm(qua);       % Brute-force normalization

%           qua_error = [kf.deltaxp(1:3)/2; 1];
%           qua = qua + qua_error;
%           qua = qua / norm(qua);
          
          % DCM correction
          DCMbn = qua2dcm(qua);

          % Euler correction
          euler = qua2euler(qua);
          roll(i) = euler(1);
          pitch(i)= euler(2);
          yaw(i)  = euler(3);

         
          % Biases estimation
          gb_dyn   = kf.deltaxp(4:6);
         
          % Store Kalman filter outputs
          nav.t(i)              = imu.t(i); 
          nav.roll(i)           = rad2deg(roll(i));   
          nav.pitch(i)          = rad2deg(pitch(i));    
          nav.yaw(i)            = rad2deg(yaw(i));
          nav.qua(i,:)          = qua;
          nav.DCMbn(i,:)        = reshape(DCMbn, 1, 9);
          nav.deltaxi(i,:)      = kf.deltaxi';
          nav.deltaxp(i,:)      = kf.deltaxp';
          nav.Phi(i,:)          = reshape(kf.Phi, 1, 36);
          nav.Pi(i,:)           = reshape(kf.Pi, 1, 36);
          nav.Pp(i,:)           = reshape(kf.Pp, 1, 36);
          nav.K(i,:)            = reshape(kf.K, 1, 36);
          nav.S(i,:)            = reshape(kf.S, 1, 36);
          nav.ob(i,:)           = ob;
          nav.deltar(i,:)       = kf.deltar;
          nav.wb(i,:)           = wb_corrected;

end


end