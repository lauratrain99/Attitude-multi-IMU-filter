function [wv] = w_imu2cm(DCMbv, wb)
% w_imu2cm converts the angular velocity data for an IMU located 
% at a position of the body different from the center of mass to
% the angular velocity felt at the center of mass of the body
%
% INPUT:
%          DCMbv, 3x3 DCM from IMU body reference frame to the center
%                 of mass reference frame  
%             wb, 3x1 angular velocity of the IMU in b coordinates 
%                 [rad/s]
% OUTPUT:
%             wv, 3x1 angular velocity of the center of mass in 
%                 v coordinates (radians/s)
%
%%
    wv = DCMbv * wb;
    
end

