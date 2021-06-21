function [mv] = mag_imu2cm(DCMbv, mb)
% mag_imu2cm converts the magnetic field data for an IMU located
% at a position of the body different from the center of mass to
% the magnetic field felt at the center of mass of the body
%
% INPUT:
%          DCMbv, 3x3 DCM from IMU body reference frame to the center
%                 of mass reference frame  
%             mb, 3x1 magnetic field of the IMU in b coordinates [Gauss]
% OUTPUT:
%             mv, 3x1 magnetic field of the center of mass in 
%                 v coordinates [Gauss]
%
%%
    mv = DCMbv * mb;
    
end

