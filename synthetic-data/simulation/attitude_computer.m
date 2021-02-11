function [quat, euler] = attitude_computer(imu)
% attitude_computer computes the attitude of a body given the values for
% tha angular velocity
%
% INPUT
%             imu, IMU data structure
%             t: Nx1 time vector (seconds).
%     ini_align: 1x3 initial attitude at t(1).
%            wv: Nx3 turn rates vector in body frame XYZ (radians/s).
% OUTPUT
%           quat, Nx4 quaternions [q1, q2, q3, q4]
%          euler, Nx3 Euler angles in radians
%
%%
    wv = imu.wv;
    t = imu.t;
    
    % initialization
    euler = [imu.ini_align(1), imu.ini_align(2), imu.ini_align(3)];
    quat = euler2qua(euler)';
    
    % get attitude from angular velocity integration
    for i = 2:length(t)
        dt = t(i) - t(i-1);
        [quat(i,:), ~, euler(i,:)] = my_quat_update(wv(i,:), quat(i-1,:)', dt);
    end
end

