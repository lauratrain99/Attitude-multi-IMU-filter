function [imuI] = VIMU_to_IMU(imuMAIN, imuI)
%
% VIMU_to_IMU converts the data for an IMU (VIMU) located at the center of mass of
% a body into the data for an IMU located at a different position
%
% INPUT
%           imuMAIN, IMU at the center of mass data structure (VIMU)
%             t: Nx1 time vector (seconds).
%            fv: Nx3 accelerations vector in body frame XYZ of the VIMU (m/s^2).
%            wv: Nx3 turn rates vector in body frame XYZ of the VIMU(radians/s).
%            mv: Nx3 magnetic field vector in NED frame of the VIMU (Gauss)
%
%           imuI, IMU located at a different position
%         DCMvb: 3x3 DCM from the VIMU reference frame to the IMU body reference frame 
%           Rvb: 3x1 lever arm from the center of mass to the location of the IMU
%                    in VIMU reference frame
% OUTPUT
%           imuMAIN, IMU at the center of mass data structure (VIMU)
%         alpha: Nx3 angular acceleration of the VIMU in VIMU reference frame (rad/s^2)
%
%          imuN, IMU located at a different position
%             t: Nx1 time vector (seconds).
%         DCMvb: 3x3 DCM from the VIMU reference frame to the imuN body reference frame 
%            wb: Nx3 angular velocity in imuN body reference frame (radians)
%            fb: Nx3 accelerations vector in imuN body reference frame (m/s^2)
%            mb: Nx3 magnetic field vector in NED frame of the imuN body
%            reference frame (Gauss)
%  
%%

    N = length(imuMAIN.t);
    imuI.t = imuMAIN.t;
    for i = 1:N
        imuI.wb(i,:) = imuI.DCMvb * imuMAIN.wv(i,:)';
        imuI.mb(i,:) = imuI.DCMvb * imuMAIN.mv(i,:)';
        
        if i == 1
            imuMAIN.alpha(i,:) = zeros(1,3);
        else
            dt = imuMAIN.t(i) - imuMAIN.t(i-1);
            imuMAIN.alpha(i,:) = (imuMAIN.wv(i,:) - imuMAIN.wv(i-1,:))/dt;
        end
         %imuI.fb(i,:) = imuI.DCMvb *(cross(imuMAIN.alpha(i,:)', imuI.Rvb)) + imuI.DCMvb *(cross(imuMAIN.wv(i,:)', cross(imuMAIN.wv(i,:)', imuI.Rvb)));
        imuI.fb(i,:) = imuI.DCMvb * imuMAIN.fv(i,:)' + imuI.DCMvb *(cross(imuMAIN.alpha(i,:)', imuI.Rvb)) + imuI.DCMvb *(cross(imuMAIN.wv(i,:)', cross(imuMAIN.wv(i,:)', imuI.Rvb)));
    end
    
end
