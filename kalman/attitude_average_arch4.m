function [navCM_datafusion] = attitude_average_arch4(navCM)

% attitude_average averages the data obtained from the EKF applied for the
% data tranformed from the IMU motion to the center of mass
%
% INPUT
%          nav1CM, EKF output from IMU 1 at the center of mass
%          nav2CM, EKF output from IMU 2 at the center of mass
%          nav3CM, EKF output from IMU 3 at the center of mass
%          nav4CM, EKF output from IMU 4 at the center of mass
%
%          The four data structures contain the following fields:
%        roll: Nx1 roll angle in degrees
%       pitch: Nx1 pitch angle in degrees
%         yaw: Nx1 yaw angle in degrees
%         qua: Nx4 quaternions
% OUTPUT
%          imuCM, data fused at the center of mass
%        roll: Nx1 averaged roll angle in degrees
%       pitch: Nx1 averaged pitch angle in degrees
%         yaw: Nx1 averaged yaw angle in degrees
%         qua: Nx4 averaged quaternions   
%           t: Nx1 time span in seconds
%  
%%
    navCM_datafusion.roll = (navCM.roll1 + navCM.roll2 + navCM.roll3 + navCM.roll4)/4;
    navCM_datafusion.pitch = (navCM.pitch1 + navCM.pitch2 + navCM.pitch3 + navCM.pitch4)/4;
    navCM_datafusion.yaw = (navCM.yaw1 + navCM.yaw2 + navCM.yaw3 + navCM.yaw4)/4;
    navCM_datafusion.qua = (navCM.qua1 + navCM.qua2 + navCM.qua3 + navCM.qua4)/4;
    navCM_datafusion.t = navCM.t;
    
end