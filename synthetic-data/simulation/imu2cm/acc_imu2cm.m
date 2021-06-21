function [fv] = acc_imu2cm(fb, DCMbv, Rvb, wv, wv_prev, dt)
% acc_imu2cm converts the acceleration data for an IMU located
% at a position of the body different from the center of mass
% to the acceleration felt at the center of mass of the body
%
% INPUT:
%             fb, 3x1 accelerations vector in the IMU body 
%                 reference frame [m/s^2]
%          DCMbv, 3x3 DCM from IMU body reference frame to 
%                 the center of mass reference frame
%            Rvb, 3x1 lever arm from the center of mass to the 
%                 location of the IMU in center of mass reference frame
%             wv, 3x1 angular velocity in the center of mass 
%                 reference frame [rad/s]
%        wv_prev, 3x1 angular velocity of the previous time iteration
%                 in the center of mass reference frame [rad/s]
%             dt, interval of time [s]
% OUTPUT:
%             fv, 3x1 accelerations vector of the center of mass 
%                 in v coordinates [m/s^2]
%
%%
    if dt == 0
        alpha = zeros(3,1);
    else
        alpha = (wv - wv_prev)/dt;
    end

    fv = DCMbv * fb - (cross(alpha, Rvb)) - ...
        (cross(wv, cross(wv, Rvb)));
        
end

