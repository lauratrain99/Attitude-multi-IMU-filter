function [imu] = IMU_simulator(imu)
   
% IMU_simulator generates synthetic data for the acceleration and local
% magnetic field given the angular velocity. It projects the gravity and
% magnetic field vector into the body frame
%
% INPUT
%             imu, IMU data structure
%             t: Nx1 time vector (seconds).
%     ini_align: 1x3 initial attitude at t(1).
%            wv: Nx3 turn rates vector in body frame XYZ (radians/s).
% OUTPUT
%             imu, IMU data structure
%             t: Nx1 time vector (seconds)
%            fv: Nx3 accelerations vector in body frame XYZ  (m/s^2).
%            mv: Nx3 magnetic field vector in NED frame (Gauss)
%  
%%
    N = length(imu.t);
    [quat, ~] = attitude_computer(imu);

    for i=1:N
        qua = quat(i,:);
        imu.fv(i,1:3)= qua2dcm(qua)'*[0;0;-9.8];
    end


    for i=1:N
        qua = quat(i,:);
        imu.mv(i,1:3)= qua2dcm(qua)'*[0.22;0;0.17];
    end


end

