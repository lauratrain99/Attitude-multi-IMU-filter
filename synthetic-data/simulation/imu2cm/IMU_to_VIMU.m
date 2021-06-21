function [imuMAIN] = IMU_to_VIMU(imuI)
% IMU_to_VIMU converts an array of data for an IMU located at a 
% position of the body differentfrom the center of mass to IMU 
% (VIMU) located at the center of mass of the body
%
% INPUT:
%          imuN, IMU located at a different position
%             t: Nx1 time vector [s]
%          imuI, IMU located at a different position
%         DCMbv: 3x3 DCM from IMU body reference frame to the 
%                VIMU reference frame  
%           Rvb: 3x1 lever arm from the center of mass to the 
%                location of the IMU in VIMU reference frame
%            wb: Nx3 angular velocity in imuN body reference 
%                frame [rad/s]
%            fb: Nx3 accelerations vector in imuN body reference 
%                frame [m/s^2]
%            mb: Nx3 magnetic field vector in NED frame of the 
%                imuN body reference frame [Gauss]
% OUTPUT:
%           imuMAIN, IMU at the center of mass data structure (VIMU)
%             t: Nx1 time vector [s]
%         alpha: Nx3 angular acceleration of the VIMU in VIMU 
%                reference frame [rad/s^2]
%            fv: Nx3 accelerations vector in body frame of the VIM
%               [m/s^2]
%            wv: Nx3 turn rates vector in body frame of the VIMU
%               [rad/s]
%            mv: Nx3 magnetic field vector in NED frame of the 
%                VIMU [Gauss]
%  
%%
    % take the IMU time
    N = length(imuI.t);
    imuMAIN.t = imuI.t;
    
    for i = 1:N
        % transform angular velocity and magnetic field
        imuMAIN.wv(i,:) = imuI.DCMbv * imuI.wb(i,:)';
        imuMAIN.mv(i,:) = imuI.DCMbv * imuI.mb(i,:)';
        
        % compute angular acceleration
        if i == 1
            imuMAIN.alpha(i,:) = zeros(1,3);
        else
            dt = imuI.t(i) - imuI.t(i-1);
            imuMAIN.alpha(i,:) = (imuMAIN.wv(i,:) -  ...
                                  imuMAIN.wv(i-1,:))/dt;
        end
        
        % transform acceleration
        imuMAIN.fv(i,:) = imuI.DCMbv * imuI.fb(i,:)' - ...
        (cross(imuMAIN.alpha(i,:)', imuI.Rvb)) - ...
        (cross(imuMAIN.wv(i,:)', cross(imuMAIN.wv(i,:)', imuI.Rvb)));
    end
    
end


