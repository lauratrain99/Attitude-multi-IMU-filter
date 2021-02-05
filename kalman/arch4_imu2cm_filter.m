function [nav] = arch4_imu2cm_filter(imu1, imu2, imu3, imu4)
%
% arch4_imu2cm_filter uses an Extended Kalman filter on four IMU measurements located at a different
% positions of the body for which the attitude is desired to be computed.
% It corresponds to the design architecture number 4.
% The four IMUs have the same error characterization. 
% The IMU measurements include accelerometer, gyro and magnetometer data.
% INPUT
%           imu1, IMU data structure.
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
%          imu2, IMU 2 data structure.
%          imu3, IMU 3 data structure.
%          imu4, IMU 4 data structure.
% OUTPUT
%          nav, INS navigation estimates data structure.
%            t: Nx1 INS time vector (seconds).
%        rolln: Nx1 roll (radians)
%       pitchn: Nx1 pitch (radians)
%         yawn: Nx1 yaw (radians)
%         quan: Mx4 corrected quaternions.
%        DCMbn: Mx9 corrected DCM matrix from body-to-nav frame.
%      deltaxi: Mx24 Kalman filter a priori error states.
%      deltaxp: Mx24 Kalman filter a posteriori error states.
%          Phi: Mx576 Kalman filter transition-state matrices, one matrix per
%               row ordered by columns.
%           Pi: Mx576 Kalman filter a priori covariance matrices, one matrix
%               per row ordered by columns.
%           Pp: Mx576 Kalman filter a posteriori covariance matrices, one
%               matrix per row ordered by columns.
%            K: Mx576 Kalman filter gain matrix.
%            S: Mx576 Kalman filter innovation matrix.
%           ob: Mx1 Evolution of number of observable states.
%  deltay_prop: Mx3 Evolution of the propagated measurement error
%       deltar: Mx12 Evolution of the error residual
%       deltay: Mx12 Evolution of the estimated measurement error
%           wv: Mx3 Evolution of the corrected angular velocity

%% PREALLOCATION AND INITIALIZATION

% Constant matrices
I = eye(3);
O = zeros(3);


% Length of time vector

N = length(imu1.t);


% Preallocation of attitude vectors
nav.roll1  = zeros(N, 1);
nav.pitch1 = zeros(N, 1);
nav.yaw1   = zeros(N, 1);
nav.qua1   = zeros(N, 4);
nav.DCMbn1 = zeros(N, 9);

nav.roll2  = zeros(N, 1);
nav.pitch2 = zeros(N, 1);
nav.yaw2   = zeros(N, 1);
nav.qua2   = zeros(N, 4);
nav.DCMbn2 = zeros(N, 9);

nav.roll3  = zeros(N, 1);
nav.pitch3 = zeros(N, 1);
nav.yaw3   = zeros(N, 1);
nav.qua3   = zeros(N, 4);
nav.DCMbn3 = zeros(N, 9);

nav.roll4  = zeros(N, 1);
nav.pitch4 = zeros(N, 1);
nav.yaw4   = zeros(N, 1);
nav.qua4   = zeros(N, 4);
nav.DCMbn4 = zeros(N, 9);


% Initial attitude at time = 1
roll1  = imu1.ini_align(1);
pitch1 = imu1.ini_align(2);
yaw1   = imu1.ini_align(3);
euler1 = [roll1, pitch1, yaw1];

roll2  = imu2.ini_align(1);
pitch2 = imu2.ini_align(2);
yaw2   = imu2.ini_align(3);
euler2 = [roll2, pitch2, yaw2];

roll3  = imu3.ini_align(1);
pitch3 = imu3.ini_align(2);
yaw3   = imu3.ini_align(3);
euler3 = [roll3, pitch3, yaw3];

roll4  = imu4.ini_align(1);
pitch4 = imu4.ini_align(2);
yaw4   = imu4.ini_align(3);
euler4 = [roll4, pitch4, yaw4];


nav.roll1(1)  = imu1.ini_align(1);
nav.pitch1(1) = imu1.ini_align(2);
nav.yaw1(1)   = imu1.ini_align(3);
nav.roll2(1)  = imu2.ini_align(1);
nav.pitch2(1) = imu2.ini_align(2);
nav.yaw2(1)   = imu2.ini_align(3);
nav.roll3(1)  = imu3.ini_align(1);
nav.pitch3(1) = imu3.ini_align(2);
nav.yaw3(1)   = imu3.ini_align(3);
nav.roll4(1)  = imu4.ini_align(1);
nav.pitch4(1) = imu4.ini_align(2);
nav.yaw4(1)   = imu4.ini_align(3);

qua1 = euler2qua(euler1);
DCMnb1 = euler2dcm(euler1);
DCMbn1 = DCMnb1';

qua2 = euler2qua(euler2);
DCMnb2 = euler2dcm(euler2);
DCMbn2 = DCMnb2';

qua3 = euler2qua(euler3);
DCMnb3 = euler2dcm(euler3);
DCMbn3 = DCMnb3';

qua4 = euler2qua(euler4);
DCMnb4 = euler2dcm(euler4);
DCMbn4 = DCMnb4';


nav.qua1(1,:) = qua1;
nav.DCMbn1(1,:) = reshape(DCMbn1, 1, 9);

nav.qua2(1,:) = qua2;
nav.DCMbn2(1,:) = reshape(DCMbn2, 1, 9);

nav.qua3(1,:) = qua3;
nav.DCMbn3(1,:) = reshape(DCMbn3, 1, 9);

nav.qua4(1,:) = qua4;
nav.DCMbn4(1,:) = reshape(DCMbn4, 1, 9);


% Initial dynamic bias
gb_dyn_1 = imu1.gb_dyn';
gb_dyn_2 = imu2.gb_dyn';
gb_dyn_3 = imu3.gb_dyn';
gb_dyn_4 = imu4.gb_dyn';

% Preallocation of Kalman filter matrices for later performance analysis
nav.t = zeros(N,1);             % Discrete time for the results
nav.deltaxi = zeros(N, 24);      % Evolution of Kalman filter a priori states, xi
nav.deltaxp = zeros(N, 24);      % Evolution of Kalman filter a posteriori states, xp
nav.Phi  = zeros(N, 576);        % Transition-state matrices, Phi
nav.Pi = zeros(N, 576);          % A priori covariance matrices, Pi
nav.Pp = zeros(N, 576);          % A posteriori covariance matrices, Pp
nav.K  = zeros(N, 576);          % Kalman gain matrices, K
nav.S  = zeros(N, 576);           % Innovation matrices, S
nav.ob = zeros(N, 1);           % Number of observable states at each acceleromter data
nav.deltar = zeros(N, 24);        % Error residual
nav.wv = zeros(N, 3);           % Corrected angular velocity

% Prior estimates for initial update
kf.deltaxi = [zeros(1,3), imu1.gb_dyn, zeros(1,3), imu2.gb_dyn, zeros(1,3), imu3.gb_dyn, zeros(1,3), imu4.gb_dyn]';            % Error vector state
kf.Pi = diag([imu1.ini_align_err, imu1.gb_dyn, imu2.ini_align_err, imu2.gb_dyn, imu3.ini_align_err, imu3.gb_dyn, imu4.ini_align_err, imu4.gb_dyn].^2);

ge = 9.81;
mN = 0.22;
mD = 0.17;
g_n = [0; 0; ge];
m_n = [mN; 0; mD];

wv_1 = w_imu2cm(imu1.DCMbv, imu1.wb(1,:)');
wv_2 = w_imu2cm(imu2.DCMbv, imu2.wb(1,:)');
wv_3 = w_imu2cm(imu3.DCMbv, imu3.wb(1,:)');
wv_4 = w_imu2cm(imu4.DCMbv, imu4.wb(1,:)');
av_1 = acc_imu2cm(imu1.fb(1,:)', imu1.DCMbv, imu1.Rvb, wv_1, wv_1, 0);
av_2 = acc_imu2cm(imu2.fb(1,:)', imu2.DCMbv, imu2.Rvb, wv_2, wv_2, 0);
av_3 = acc_imu2cm(imu3.fb(1,:)', imu3.DCMbv, imu3.Rvb, wv_3, wv_3, 0);
av_4 = acc_imu2cm(imu4.fb(1,:)', imu4.DCMbv, imu4.Rvb, wv_4, wv_4, 0);
mv_1 = mag_imu2cm(imu1.DCMbv, imu1.mb(1,:)');
mv_2 = mag_imu2cm(imu2.DCMbv, imu2.mb(1,:)');
mv_3 = mag_imu2cm(imu3.DCMbv, imu3.mb(1,:)');
mv_4 = mag_imu2cm(imu4.DCMbv, imu4.mb(1,:)');

kf.deltar = zeros(24,1);
kf.deltar(1:6) = [-DCMbn1*av_1 - g_n; DCMbn1*mv_1 - m_n]';
kf.deltar(7:12) = [-DCMbn2*av_2 - g_n; DCMbn2*mv_2 - m_n]';
kf.deltar(13:18) = [-DCMbn3*av_3 - g_n; DCMbn3*mv_3 - m_n]';
kf.deltar(19:24) = [-DCMbn4*av_4 - g_n; DCMbn4*mv_4 - m_n]';

% Measurement matrix, constant value over time
kf.H = [skewm(g_n) O O O O O O O;
        skewm(m_n) O O O O O O O;
        O O skewm(g_n) O O O O O;
        O O skewm(m_n) O O O O O;
        O O O O skewm(g_n) O O O;
        O O O O skewm(m_n) O O O;
        O O O O O O skewm(g_n) O;
        O O O O O O skewm(m_n) O];


% Correction covariance matrix, constant value over time
kf.R = diag([imu1.a_std, imu1.m_std, imu2.a_std, imu2.m_std, imu3.a_std, imu3.m_std, imu4.a_std, imu4.m_std]).^2;

% Propagate prior estimates to get xp(1) and Pp(1)
kf = kf_update( kf );


% Initial matrices for Kalman filter performance analysis
nav.deltaxi(1,:) = kf.deltaxi;
nav.Pi = reshape(kf.Pi, 1, 576);

nav.deltaxp(1,:) = kf.deltaxp;
nav.Pp(1,:) = reshape(kf.Pp, 1, 576);
nav.deltar(1,:) = kf.deltar;
nav.wv(1,:) = wv_1;


% Prediction covariance matrix, constant value over time
init_alpha_1 = imu1.DCMbv*imu1.g_std';
init_alpha_2 = imu2.DCMbv*imu2.g_std';
init_alpha_3 = imu3.DCMbv*imu3.g_std';
init_alpha_4 = imu4.DCMbv*imu4.g_std';
kf.Q  = diag([init_alpha_1', imu1.gb_dyn, init_alpha_2', imu2.gb_dyn, init_alpha_3', imu3.gb_dyn, init_alpha_4', imu4.gb_dyn].^2);


for i = 2:N

      % IMU sampling interval
      dt = imu1.t(i) - imu1.t(i-1);
     
      % Correction for angular velocity with bias inestability
      wv_corrected_1 = w_imu2cm(imu1.DCMbv, imu1.wb(i,:)' - gb_dyn_1);
      wv_corrected_2 = w_imu2cm(imu2.DCMbv, imu2.wb(i,:)' - gb_dyn_2);
      wv_corrected_3 = w_imu2cm(imu3.DCMbv, imu3.wb(i,:)' - gb_dyn_3);
      wv_corrected_4 = w_imu2cm(imu4.DCMbv, imu4.wb(i,:)' - gb_dyn_4);
      
      % Attitude update 3-2-1 body sequence
      [qua1, DCMbn1, ~] = my_quat_update(wv_corrected_1, qua1, dt);
      [qua2, DCMbn2, ~] = my_quat_update(wv_corrected_2, qua2, dt);
      [qua3, DCMbn3, ~] = my_quat_update(wv_corrected_3, qua3, dt);
      [qua4, DCMbn4, ~] = my_quat_update(wv_corrected_4, qua4, dt);
      
      % KALMAN FILTER      
          % PREDICTION         
          kf.F = [-skewm(wv_corrected_1) -I O O O O O O;
                   O O O O O O O O;
                   O O -skewm(wv_corrected_2) -I O O O O;
                   O O O O O O O O;
                   O O O O -skewm(wv_corrected_3) -I O O;
                   O O O O O O O O;
                   O O O O O O -skewm(wv_corrected_4) -I;
                   O O O O O O O O];
       
          %kf.J = [-DCMbn -I O; O O I];
          kf.G = [-I O O O O O O O;
                   O I O O O O O O;
                   O O -I O O O O O;
                   O O O I O O O O;
                   O O O O -I O O O;
                   O O O O O I O O;
                   O O O O O O -I O;
                   O O O O O O O I];
          
          % CORRECTION
          wv_prev = nav.wv(i-1,:)';
          
          wv_1 = w_imu2cm(imu1.DCMbv, imu1.wb(i,:)');
          wv_2 = w_imu2cm(imu2.DCMbv, imu2.wb(i,:)');
          wv_3 = w_imu2cm(imu3.DCMbv, imu3.wb(i,:)');
          wv_4 = w_imu2cm(imu4.DCMbv, imu4.wb(i,:)');
          av_1 = acc_imu2cm(imu1.fb(i,:)', imu1.DCMbv, imu1.Rvb, wv_1, wv_prev, 0);
          av_2 = acc_imu2cm(imu2.fb(i,:)', imu2.DCMbv, imu2.Rvb, wv_2, wv_prev, 0);
          av_3 = acc_imu2cm(imu3.fb(i,:)', imu3.DCMbv, imu3.Rvb, wv_3, wv_prev, 0);
          av_4 = acc_imu2cm(imu4.fb(i,:)', imu4.DCMbv, imu4.Rvb, wv_4, wv_prev, 0);
          mv_1 = mag_imu2cm(imu1.DCMbv, imu1.mb(i,:)');
          mv_2 = mag_imu2cm(imu2.DCMbv, imu2.mb(i,:)');
          mv_3 = mag_imu2cm(imu3.DCMbv, imu3.mb(i,:)');
          mv_4 = mag_imu2cm(imu4.DCMbv, imu4.mb(i,:)');
          
          kf.deltar = zeros(24,1);
          kf.deltar(1:6) = [-DCMbn1*av_1 - g_n; DCMbn1*mv_1 - m_n]';
          kf.deltar(7:12) = [-DCMbn2*av_2 - g_n; DCMbn2*mv_2 - m_n]';
          kf.deltar(13:18) = [-DCMbn3*av_3 - g_n; DCMbn3*mv_3 - m_n]';
          kf.deltar(19:24) = [-DCMbn4*av_4 - g_n; DCMbn4*mv_4 - m_n]';
          
          % Execution of the Extended Kalman filter
          kf.deltaxp = zeros(length(kf.deltaxp),1);           % states 1:3 are forced to be zero (error-state approach)
          kf = kalman(kf, dt);
         
          % OBSERVABILITY
          % Number the observable states
          ob = rank(obsv(kf.F, kf.H));
         
          % ADD THE ERROR TO THE STATE
          % Quaternion corrections
          % Crassidis. Eq. 7.34 and A.174a.
          antm = [0 qua1(3) -qua1(2); -qua1(3) 0 qua1(1); qua1(2) -qua1(1) 0];
          qua1 = qua1 + 0.5 .* [qua1(4)*eye(3) + antm; -1.*[qua1(1) qua1(2) qua1(3)]] * kf.deltaxp(1:3);
          qua1 = qua1 / norm(qua1);       % Brute-force normalization
          
          antm = [0 qua2(3) -qua2(2); -qua2(3) 0 qua2(1); qua2(2) -qua2(1) 0];
          qua2 = qua2 + 0.5 .* [qua2(4)*eye(3) + antm; -1.*[qua2(1) qua2(2) qua2(3)]] * kf.deltaxp(7:9);
          qua2 = qua2 / norm(qua2);       % Brute-force normalization
          
          antm = [0 qua3(3) -qua3(2); -qua3(3) 0 qua3(1); qua3(2) -qua3(1) 0];
          qua3 = qua3 + 0.5 .* [qua3(4)*eye(3) + antm; -1.*[qua3(1) qua3(2) qua3(3)]] * kf.deltaxp(13:15);
          qua3 = qua3 / norm(qua3);       % Brute-force normalization
          
          antm = [0 qua4(3) -qua4(2); -qua4(3) 0 qua4(1); qua4(2) -qua4(1) 0];
          qua4 = qua4 + 0.5 .* [qua4(4)*eye(3) + antm; -1.*[qua4(1) qua4(2) qua4(3)]] * kf.deltaxp(19:21);
          qua4 = qua4 / norm(qua4);       % Brute-force normalization


          
          % DCM correction
          DCMbn1 = qua2dcm(qua1);
          DCMbn2 = qua2dcm(qua2);
          DCMbn3 = qua2dcm(qua3);
          DCMbn4 = qua2dcm(qua4);

          % Euler correction
          euler1 = qua2euler(qua1);
          roll1 = euler1(1);
          pitch1= euler1(2);
          yaw1  = euler1(3);
          
          euler2 = qua2euler(qua2);
          roll2 = euler2(1);
          pitch2= euler2(2);
          yaw2  = euler2(3);
          
          euler3 = qua2euler(qua3);
          roll3 = euler3(1);
          pitch3= euler3(2);
          yaw3  = euler3(3);
          
          euler4 = qua2euler(qua4);
          roll4 = euler4(1);
          pitch4= euler4(2);
          yaw4  = euler4(3);

         
          % Biases estimation
          gb_dyn_1 = kf.deltaxp(4:6);
          gb_dyn_2 = kf.deltaxp(10:12);
          gb_dyn_3 = kf.deltaxp(16:18);
          gb_dyn_4 = kf.deltaxp(22:24);

         
          % Store Kalman filter outputs
          nav.t(i)              = imu1.t(i); 
          nav.roll1(i)           = rad2deg(roll1);   
          nav.pitch1(i)          = rad2deg(pitch1);    
          nav.yaw1(i)            = rad2deg(yaw1);
          nav.roll2(i)           = rad2deg(roll2);   
          nav.pitch2(i)          = rad2deg(pitch2);    
          nav.yaw2(i)            = rad2deg(yaw2);
          nav.roll3(i)           = rad2deg(roll3);   
          nav.pitch3(i)          = rad2deg(pitch3);    
          nav.yaw3(i)            = rad2deg(yaw3);
          nav.roll4(i)           = rad2deg(roll4);   
          nav.pitch4(i)          = rad2deg(pitch4);    
          nav.yaw4(i)            = rad2deg(yaw4);
          nav.qua1(i,:)          = qua1;
          nav.qua2(i,:)          = qua2;
          nav.qua3(i,:)          = qua3;
          nav.qua4(i,:)          = qua4;
          nav.DCMbn1(i,:)        = reshape(DCMbn1, 1, 9);
          nav.DCMbn2(i,:)        = reshape(DCMbn2, 1, 9);
          nav.DCMbn3(i,:)        = reshape(DCMbn3, 1, 9);
          nav.DCMbn4(i,:)        = reshape(DCMbn4, 1, 9);
          nav.deltaxi(i,:)      = kf.deltaxi';
          nav.deltaxp(i,:)      = kf.deltaxp';
          nav.Phi(i,:)          = reshape(kf.Phi, 1, 576);
          nav.Pi(i,:)           = reshape(kf.Pi, 1, 576);
          nav.Pp(i,:)           = reshape(kf.Pp, 1, 576);
          nav.K(i,:)            = reshape(kf.K, 1, 576);
          nav.S(i,:)            = reshape(kf.S, 1, 576);
          nav.ob(i,:)           = ob;
          nav.deltar(i,:)       = kf.deltar;
          nav.wv_1(i,:)           = wv_corrected_1;
          nav.wv_2(i,:)           = wv_corrected_2;
          nav.wv_3(i,:)           = wv_corrected_3;
          nav.wv_4(i,:)           = wv_corrected_4;

end


end