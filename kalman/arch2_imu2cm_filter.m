function [nav, imuCM] = arch2_imu2cm_filter(imu)
%
% multiple_imu_filter uses an Extended Kalman filter on four IMU measurements located at different
% positions of the body for which the attitude is desired to be computed.
% It corresponds to the design architecture number 2.
% The four IMUs have the same error characterization
% The IMU measurements include accelerometer, gyro and magnetometer data.
% INPUT
%          imu1, IMU located at position N, different from the center of mass
%             t: Nx1 time vector (seconds).
%            fb: Nx3 accelerations vector in the imu body reference frame, XYZ coords (m/s^2)
%            wb: Nx3 turn rates vector in the imu body reference frame, XYZ coords (radians/s)
%            mb: Nx3 magnetic field in the imu body reference frame, NED coords (Gauss)
%         g_std: Nx3 gyros standard deviations (radians/s)
%         a_std: Nx3 accrs standard deviations (m/s^2)
%         m_std: Nx3 magns standard deviations (Gauss)
%        gb_dyn: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%     ini_align: 1x3 initial attitude at t(1).
% ini_align_err: 1x3 initial attitude errors at t(1).
%

% OUTPUT
%          nav, INS navigation estimates data structure.
%            t: Nx1 INS time vector (seconds).
%         roll: Nx1 roll (radians).
%        pitch: Nx1 pitch (radians).
%          yaw: Nx1 yaw (radians).
%          qua: Nx4 corrected quaternions.
%        DCMbn: Nx9 corrected DCM matrix from body-to-nav frame.
%      deltaxi: Nx6 Kalman filter a priori error states.
%      deltaxp: Nx6 Kalman filter a posteriori error states.
%          Phi: Nx36 Kalman filter transition-state matrices, one matrix per
%               row ordered by columns.
%           Pi: Nx36 Kalman filter a priori covariance matrices, one matrix
%               per row ordered by columns.
%           Pp: Nx36 Kalman filter a posteriori covariance matrices, one
%               matrix per row ordered by columns.
%            K: Mx36 Kalman filter gain matrix.
%            S: Nx36 Kalman filter innovation matrix.
%           ob: Nx1 Evolution of number of observable states.
%  deltay_prop: Nx3 Evolution of the propagated measurement error
%       deltar: Nx3 Evolution of the error residual
%       deltay: Nx3 Evolution of the estimated measurement error
%           wv: Nx3 Evolution of the corrected angular velocity
%   
%     imuMAINcomputed, data structure fused at the center of mass
%          wv: Nx3 turn rates vector in body frame XYZ of the VIMU(radians/s)
%          fv: Nx3 accelerations vector in body frame XYZ of the VIMU (m/s^2)
%          mv: Nx3 magnetic field vector in NED frame of the VIMU (Gauss)     



%% PREALLOCATION AND INITIALIZATION

% Constant matrices
I = eye(3);
O = zeros(3);

% Compute the equivalent ang rates, acc and magnetic field at the center of
% mass from each imu
[imuCM] = IMU_to_VIMU(imu);


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

% Convert Euler into quaternions and DCMs
qua = euler2qua(euler);
DCMnb = euler2dcm(euler);
DCMbn = DCMnb';

nav.qua(1,:) = qua;
nav.DCMbn(1,:) = reshape(DCMbn, 1, 9);


% Initial dynamic bias
gv_dyn = imu.DCMbv*imu.gb_dyn';

% Preallocation of Kalman filter matrices for later performance analysis
nav.t = zeros(N,1);             % Discrete time for the results
nav.deltaxi = zeros(N, 6);      % Evolution of Kalman filter a priori states, xi
nav.deltaxp = zeros(N, 6);      % Evolution of Kalman filter a posteriori states, xp
nav.Phi  = zeros(N, 36);        % Transition-state matrices, Phi
nav.Pi = zeros(N, 36);          % A priori covariance matrices, Pi
nav.Pp = zeros(N, 36);          % A posteriori covariance matrices, Pp
nav.K  = zeros(N, 36);          % Kalman gain matrices, K
nav.S  = zeros(N, 36);           % Innovation matrices, S
nav.ob = zeros(N, 1);           % Number of observable states at each acc/magn data
nav.deltar = zeros(N,6);        % Error residual
nav.wv = zeros(N, 3);           % Corrected angular velocity

% Prior estimates for initial update
kf.deltaxi = [zeros(1,3), gv_dyn']';            
kf.Pi = diag([imu.DCMbv*imu.ini_align_err'; gv_dyn].^2);

% Compute the initial state
ge = -9.81;
mN = 0.22;
mD = 0.17;
g_n = [0; 0; ge];
m_n = [mN; 0; mD];
av = imuCM.fv(1,:)';
mv = imuCM.mv(1,:)';
kf.deltar = [-DCMbn*av - g_n; DCMbn*mv - m_n];


% Measurement matrix, constant value over time
kf.H = [skewm(g_n) O; skewm(m_n) O];

% Correction covariance matrix, constant value over time
kf.R = diag([imu.DCMbv*imu.a_std'; imu.DCMbv*imu.m_std']).^2;

% Initialization of the KF
kf = kf_update( kf );



% Initial KF matrices and vectors
nav.deltaxi(1,:) = kf.deltaxi;
nav.Pi = reshape(kf.Pi, 1, 36);

nav.deltaxp(1,:) = kf.deltaxp;
nav.Pp(1,:) = reshape(kf.Pp, 1, 36);
nav.deltar(1,:) = kf.deltar;
nav.wv(1,:) = imuCM.wv(1,:)' - gv_dyn;


% Prediction covariance matrix, constant value over time
kf.Q  = diag([imu.DCMbv*imu.g_std'; gv_dyn].^2);


for i = 2:N

      % IMU sampling interval
      dt = imu.t(i) - imu.t(i-1);
     
      % Correction for angular velocity with bias inestability
      wv_corrected = imuCM.wv(i,:)' - gv_dyn;
     
      % Attitude update 3-2-1 body sequence
      [qua, DCMbn, euler] = my_quat_update(wv_corrected, qua, dt);

      % KALMAN FILTER      
          % PREDICTION         
          kf.F = [-skewm(wv_corrected) -I; O O];
          kf.G = [-I O; O I];
          
          % CORRECTION
          av = imuCM.fv(i,:)';
          mv = imuCM.mv(i,:)';
          kf.deltar = [-DCMbn*av - g_n; DCMbn*mv - m_n];
          
          % error reset
          kf.deltaxp = zeros(length(kf.deltaxp),1);  
          
          % Execution of the Extended Kalman filter
          kf = kalman(kf, dt);
         
          % OBSERVABILITY
          % Number the observable states
          ob = rank(obsv(kf.F, kf.H));
         
          % ADD THE ERROR TO THE STATE
          % Quaternion corrections
          % Crassidis. Eq. 7.34 and A.174a.
          antm = [0 qua(3) -qua(2); -qua(3) 0 qua(1); qua(2) -qua(1) 0];
          qua = qua + 0.5 .* [qua(4)*eye(3) + antm; -1.*[qua(1) qua(2) qua(3)]] * kf.deltaxp(1:3);
          
          % Brute-force normalization
          qua = qua / norm(qua);       
          
          % DCM correction
          DCMbn = qua2dcm(qua);

          % Euler correction
          euler = qua2euler(qua);
          roll(i) = euler(1);
          pitch(i)= euler(2);
          yaw(i)  = euler(3);

         
          % Biases estimation
          gv_dyn   = kf.deltaxp(4:6);
         
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
          nav.wv(i,:)           = wv_corrected;

end


end


