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
    [~, euler] = attitude_computer(imu);

    roll = euler(:,1);
    pitch = euler(:,2);
    yaw = euler(:,3);
    
    for i=1:N
        imu.fv(i,1:3)=euler2dcm([roll(i), pitch(i), yaw(i)])*[0;0;-9.8];
    end


    for i=1:N
        imu.mv(i,1:3)=euler2dcm([roll(i), pitch(i), yaw(i)])*[0.22;0;0.17];
    end
end


% data = [t', wb, fb, mb];
% fileID = fopen('xAxisRotation.txt','w');
% fprintf(fileID,'%6.2f %6.2f %12.8f %6.2f %12.8f %6.2f %12.8f %6.2f %6.2f %12.8f\n',data');
% fclose(fileID);

 
% mag = [0.22; 0; 1.17];
% for i=1:length(t)
% 
%     cmTerr(1:3,i)=euler2dcm([roll(i), pitch(i), yaw(i)])'*[0.22; 0; 1.17];
% 
% 
% end