function [imu] = IMU_simulator(imu)
% IMU_simulator generates synthetic data for the accelerometer
% and magnetometer given the gyro measurements
%
% INPUT
%             imu, IMU data structure
%               t: Nx1 time vector [s]
%       ini_align: 1x3 initial attitude 
%              wv: Nx3 turn rates vector in body frame [rad/s]
% OUTPUT
%             imu, IMU data structure
%               t: Nx1 time vector [s]
%              fv: Nx3 accelerations vector in body frame [m/s^2]
%              mv: Nx3 magnetic field vector in NED frame [Gauss]
%  
%%
    N = length(imu.t);
    [quat, ~] = attitude_computer(imu);
    
    % accelerometer values
    for i=1:N
        qua = quat(i,:);
        imu.fv(i,1:3)= qua2dcm(qua)'*[0;0;9.8];
    end
    
    % magnetometer values
    for i=1:N
        qua = quat(i,:);
        imu.mv(i,1:3)= qua2dcm(qua)'*[0.22;0;0.17];
    end

end

