function [imuCM] = sensor_fusion_arch1(imu1CM, imu2CM, imu3CM, imu4CM)

% sensor_fusion averages the data obtained for the center of mass of the
% body from each of the four imus
%
% INPUT
%          imu1CM, data at the center of mass computed from IMU 1 readings
%          imu2CM, data at the center of mass computed from IMU 2 readings
%          imu3CM, data at the center of mass computed from IMU 3 readings
%          imu3CM, data at the center of mass computed from IMU 4 readings
%
%          The four data structures contain the following fields:
%          wv: Nx3 turn rates vector in body frame XYZ of the VIMU(radians/s)
%          fv: Nx3 accelerations vector in body frame XYZ of the VIMU (m/s^2)
%          mv: Nx3 magnetic field vector in NED frame of the VIMU (Gauss)
% OUTPUT
%          imuCM, data fused at the center of mass
%          wv: Nx3 turn rates vector in body frame XYZ of the VIMU(radians/s)
%          fv: Nx3 accelerations vector in body frame XYZ of the VIMU (m/s^2)
%          mv: Nx3 magnetic field vector in NED frame of the VIMU (Gauss)    
%  
%%
    imuCM.wv = (imu1CM.wv + imu2CM.wv + imu3CM.wv + imu4CM.wv)/4;
    imuCM.fv = (imu1CM.fv + imu2CM.fv + imu3CM.fv + imu4CM.fv)/4;
    imuCM.mv = (imu1CM.mv + imu2CM.mv + imu3CM.mv + imu4CM.mv)/4;
    
end

