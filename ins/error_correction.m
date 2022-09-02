function [qua_n] = error_correction(delta_alpha, qua)
% error_correction applies the Crassidis book 7.34 equation for the 
% fusion of the error of the Euler angles and the computed attitude
% INPUT:
%    delta_alpha, angular error [rad]
%            qua, 4x1 quaternion 
% OUTPUT:
%          qua_n, 4x1 next iteration quaternion
%
%%
    antm = [0 qua(3) -qua(2); -qua(3) 0 qua(1); qua(2) -qua(1) 0];
    qua = qua + 0.5 .* [qua(4)*eye(3) + antm; -1.*[qua(1) qua(2) ...
                        qua(3)]] * delta_alpha;
    
    % Brute-force normalization
    qua_n = qua / norm(qua);       
    
end

