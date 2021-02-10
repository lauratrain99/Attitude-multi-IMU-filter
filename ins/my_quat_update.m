function [qua_n, DCMbn_n, euler] = my_quat_update(wb, qua, dt)
% my_quat_update: updates attitude using quaternion representation. 
% 3-2-1 body sequence
%
% INPUT:
%           wb, 3x1 incremental turn-rates in body-frame (rad/s).
%          qua, 4x1 quaternion.
%           dt, 1x1 IMU sampling interval (s).
%
% OUTPUT:
%        qua_n, 4x1 updated quaternion.
%      DCMbn_n, 3x3 updated body-to-nav DCM.
%        euler, 3x1 updated Euler angles (rad).


%% Ignore transport rate and Earth rate

    wb_n = wb ;

%% Quaternion update   

    qua_n   = qua_update(qua, wb_n, dt);    % Update quaternion
    qua_n   = qua_n / norm(qua_n);          % Brute-force normalization
    DCMbn_n = qua2dcm(qua_n);               % Update DCM
    euler   = qua2euler(qua_n);             % Update Euler angles

end

