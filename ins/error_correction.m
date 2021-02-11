function [qua_n] = error_correction(delta_alpha, qua)
% error_correction applies the Crassidis book 7.34 equation for the fusion of the
% error of the Euler angles and the computed attitude
% INPUT:
%           kf, Kalman filter data structure where the used fields
%      deltaxp: a posteriori state vector
%          qua: 4x1 quaternion from the attitude computer
% OUTPUT:
%        qua_n: 4x1 corrected quaternion
%
%%
    antm = [0 qua(3) -qua(2); -qua(3) 0 qua(1); qua(2) -qua(1) 0];
    qua = qua + 0.5 .* [qua(4)*eye(3) + antm; -1.*[qua(1) qua(2) qua(3)]] * delta_alpha;
    qua_n = qua / norm(qua);       % Brute-force normalization
    
end

