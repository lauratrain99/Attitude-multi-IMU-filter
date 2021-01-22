function  kf = kf_update(kf)
% Measurement update part of the Kalman filter algorithm.
% Use only acceleration measurements
%
% INPUT
%   kf, data structure with at least the following fields:
%  deltaxi: 6x1 a priori state vector.
%   deltay: 3x1 estimated measurement error.
%       Pi: 6x6 a priori error covariance matrix.
%        H: 3x6 observation matrix.
%        R: 3x3 observation noise covariance matrix.
%
% OUTPUT
%    kf, the following fields are updated:
%  deltaxp: 6x1 a posteriori state vector (updated).
%  deltaPp: 6x6 a posteriori error covariance matrix (updated).  
%        K: 15x6 Kalman gain matrix matrix.
%        S: 6x6 innovation (not residual) covariance matrix.
% deltay_p: 3x1 propagated measurement error (should be always  the zero vector)
%   deltar: 3x1 error residual.

%%


kf.S = (kf.R + kf.H * kf.Pi * kf.H');
kf.K = (kf.Pi * kf.H') * (kf.S)^(-1);

kf.deltaxp = kf.K * kf.deltar;

%kf.Pp = (eye(6) - kf.K * kf.H)*kf.Pi;
kf.Pp = kf.Pi - kf.K * kf.S * kf.K';
kf.Pp =  0.5 .* (kf.Pp + kf.Pp');               % Force Pi to be a symmetric matrix


end
