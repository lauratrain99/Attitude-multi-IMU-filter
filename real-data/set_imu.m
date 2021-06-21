function [imu] = set_imu(L, imu, imu_error, config)
% set_imu stores in imu structure the error characterization and rotation
% matrices ofr the imu
% INPUTS:
%               L, length from the center of mass to the IMU
%       imu_error, data structure with the fields:
%       ini_align: initial alignment of the center of mass
%            accX: data structure with the fields:
%                  meas: measurements from which the errors have been
%                  calculated [g's]
%             imu, data structure with the fields:
%           roll0: initial roll angle wrt the center of mass [deg]
%          pitch0: initial pitch angle wrt the center of mass [deg]
%            yaw0: initial yaw angle wrt the center of mass [deg]
%
%%
        % DCM from IMU1 body reference frame to the VIMU ref frame
        imu.DCMvb =  euler2dcm(deg2rad([imu.roll0, imu.pitch0, imu.yaw0]));
        imu.DCMbv = imu.DCMvb';
        
        if config == 1
            % lever arm in VIMU coordinates
            imu.Rvb = imu.DCMbv*[0, 0, L]';
        elseif config == 0
            imu.Rvb = imu.DCMbv*[0, L, 0]';
        else
            error('config should be 0 or 1')
        end


        % Initialize a priori data from the sensor
        imu.gb_dyn = [imu_error.gyroX.bias_stab, imu_error.gyroY.bias_stab ,imu_error.gyroZ.bias_stab];
        imu.a_std = [imu_error.accX.std, imu_error.accY.std, imu_error.accZ.std];
        imu.g_std = [imu_error.gyroX.std, imu_error.gyroY.std, imu_error.gyroZ.std];

    
end

