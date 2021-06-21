function [nav, kf] = arch3_imu2cm_filter_nomag(imu)
%
% arch3_imu2cm_filter uses an Extended Kalman filter on one IMU measurements located at a different
% positions of the body for which the attitude is desired to be computed.
% It corresponds to the design architecture number 3.
% The four IMUs have the same error characterization
% The IMU measurements include accelerometer, gyro and magnetometer data.
% INPUT
%           imu, IMU data structure.
%             t: Nx1 time vector (seconds).
%            fb: Nx3 accelerations vector of the IMU in b coords (m/s^2)
%            wb: Nx3 turn rates vector of the IMU in b coords (radians/s)
%            mb: Nx3 magnetic field of the IMU in b coords (Gauss)
%         g_std: Nx3 gyros standard deviations (radians/s)
%         a_std: Nx3 accrs standard deviations (m/s^2)
%         m_std: Nx3 magns standard deviations (Gauss)
%        gb_dyn: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%     ini_align: 1x3 initial attitude at t(1).
% ini_align_err: 1x3 initial attitude errors at t(1).
%           Rvb: 3x1 lever arm from the center of mass to the location of the IMU
%                    in center of mass reference frame
%         DCMbv: 3x3 DCM from IMU body reference frame to the center of mass reference frame
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
%           wv: Mx3 Evolution of the corrected angular velocity

%% PREALLOCATION AND INITIALIZATION

% Constant matrices
I = eye(3);
O = zeros(3);


% Length of time vector

N = length(imu.t);


% Preallocation of attitude vectors
nav.roll  = zeros (N, 1);
nav.pitch = zeros (N, 1);
nav.yaw   = zeros (N, 1);
nav.qua   = zeros(N, 4);
nav.DCMbn = zeros(N, 9);


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
nav.t = zeros(N,1);             % Discrete time for the results
nav.deltaxi = zeros(N, 6);      % Evolution of Kalman filter a priori states, xi
nav.deltaxp = zeros(N, 6);      % Evolution of Kalman filter a posteriori states, xp
nav.Phi  = zeros(N, 36);        % Transition-state matrices, Phi
nav.Pi = zeros(N, 36);          % A priori covariance matrices, Pi
nav.Pp = zeros(N, 36);          % A posteriori covariance matrices, Pp
nav.K  = zeros(N, 18);          % Kalman gain matrices, K
nav.S  = zeros(N, 9);           % Innovation matrices, S
nav.ob = zeros(N, 1);           % Number of observable states at each acceleromter data
nav.deltar = zeros(N,3);        % Error residual
nav.wv = zeros(N, 3);           % Corrected angular velocity

% Prior estimates for initial update
kf.deltaxi = [zeros(1,3), imu.gb_dyn]';            % Error vector state
kf.Pi = diag([imu.ini_align_err, imu.gb_dyn].^2);

ge = -9.81;
g_n = [0; 0; ge];

wv = w_imu2cm(imu.DCMbv, imu.wb(1,:)');
av = acc_imu2cm(imu.fb(1,:)', imu.DCMbv, imu.Rvb, wv, wv, 0);
kf.deltar = [-DCMbn*av - g_n];



% Measurement matrix, constant value over time
kf.H = [skewm(g_n) O];

% Correction covariance matrix, constant value over time
kf.R = diag([imu.a_std]).^2;

% Propagate prior estimates to get xp(1) and Pp(1)
kf = kf_update( kf );



% Initial matrices for Kalman filter performance analysis
nav.deltaxi(1,:) = kf.deltaxi;
nav.Pi = reshape(kf.Pi, 1, 36);

nav.deltaxp(1,:) = kf.deltaxp;
nav.Pp(1,:) = reshape(kf.Pp, 1, 36);
nav.deltar(1,:) = kf.deltar;
nav.wv(1,:) = wv;


% Prediction covariance matrix, constant value over time
init_alpha = imu.DCMbv*imu.g_std';
kf.Q  = diag([init_alpha', imu.gb_dyn].^2);


for i = 2:N

      % IMU sampling interval
      dt = imu.t(i) - imu.t(i-1);
     
      % Correction for angular velocity with bias inestability
      wv_corrected = w_imu2cm(imu.DCMbv, imu.wb(i,:)' - gb_dyn);
     
      % Attitude update 3-2-1 body sequence
      [qua, DCMbn, ~] = my_quat_update(wv_corrected, qua, dt);

      % KALMAN FILTER      
          % PREDICTION         
          kf.F = [-skewm(wv_corrected) -I; O O];
          kf.G = [-I O; O I];
          
          % CORRECTION
          wv_prev = nav.wv(i-1,:)';
          wv = w_imu2cm(imu.DCMbv, imu.wb(i,:)');
          av = acc_imu2cm(imu.fb(i,:)', imu.DCMbv, imu.Rvb, wv, wv_prev, dt);
          kf.deltar = [-DCMbn*av - g_n];
          
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
          nav.K(i,:)            = reshape(kf.K, 1, 18);
          nav.S(i,:)            = reshape(kf.S, 1, 9);
          nav.ob(i,:)           = ob;
          nav.deltar(i,:)       = kf.deltar;
          nav.wv(i,:)           = wv_corrected;

end


end


