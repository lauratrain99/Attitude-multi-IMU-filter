function [navCM] = attitude_average(nav1CM, nav2CM, nav3CM, nav4CM)

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
    navCM.roll = (nav1CM.roll + nav2CM.roll + nav3CM.roll + nav4CM.roll)/4;
    navCM.pitch = (nav1CM.pitch + nav2CM.pitch + nav3CM.pitch + nav4CM.pitch)/4;
    navCM.yaw = (nav1CM.yaw + nav2CM.yaw + nav3CM.yaw + nav4CM.yaw)/4;
    navCM.qua = (nav1CM.qua + nav2CM.qua + nav3CM.qua + nav4CM.qua)/4;
    navCM.t = nav1CM.t;
    
end

