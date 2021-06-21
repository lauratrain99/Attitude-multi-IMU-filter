function  kf = kf_prediction(kf, dt)
% Prediction update part of the Kalman filter algorithm.
%
% INPUT
%   kf, data structure with at least the following fields:
%       xp: 6x1 a posteriori state vector (old).
%       Pp: 6x6 a posteriori error covariance matrix (old).
%        F: 6x6 state transition matrix.
%        Q: 9x9 process noise covariance matrix.
%        J: 6x9 noise matrix.    
%   	dt: sampling interval. 
%
% OUTPUT
%   kf, the following fields are updated:
%       xi: 6x1 a priori state vector (updated).
%       Pi: 6x6 a priori error covariance matrix.
%      Phi: 6x6 state transition matrix.
%       Qd: 6x6 discrete process noise covariance matrix.

%%

    kf.Phi = expm(kf.F .* dt);
    kf.deltaxi = kf.Phi * kf.deltaxp;
    
    kf.Qd = (kf.G * kf.Q * kf.G') .* dt; 
    kf.Pi = kf.Phi * kf.Pp * kf.Phi' + kf.Qd;
    
    % Force Pi to be symmetric matrix
    kf.Pi =  0.5 .* (kf.Pi + kf.Pi');               


end
