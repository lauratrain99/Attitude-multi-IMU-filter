function [imu1, imu2, imu3, imu4] = read_4imu(data)
    
    % get IMU 1 data
    imu1_data = data(data(:,1) == 1, :);
    imu1.t = imu1_data(:,2)/1000;
    imu1.accX.meas = imu1_data(:,3)*9.81;
    imu1.accY.meas = imu1_data(:,4)*9.81;
    imu1.accZ.meas = imu1_data(:,5)*9.81;
    imu1.gyroX.meas = deg2rad(imu1_data(:,6));
    imu1.gyroY.meas = deg2rad(imu1_data(:,7));
    imu1.gyroZ.meas = deg2rad(imu1_data(:,8));
    imu1.magX.meas = imu1_data(:,9)*0.01;
    imu1.magY.meas = imu1_data(:,10)*0.01;
    imu1.magZ.meas = imu1_data(:,11)*0.01;

    % get IMU 2 data
    imu2_data = data(data(:,1) == 2, :);
    imu2.t = imu2_data(:,2)/1000;
    imu2.accX.meas = imu2_data(:,3)*9.81;
    imu2.accY.meas = imu2_data(:,4)*9.81;
    imu2.accZ.meas = imu2_data(:,5)*9.81;
    imu2.gyroX.meas = deg2rad(imu2_data(:,6));
    imu2.gyroY.meas = deg2rad(imu2_data(:,7));
    imu2.gyroZ.meas = deg2rad(imu2_data(:,8));
    imu2.magX.meas = imu2_data(:,9)*0.01;
    imu2.magY.meas = imu2_data(:,10)*0.01;
    imu2.magZ.meas = imu2_data(:,11)*0.01;
    
    % get IMU 3 data
    imu3_data = data(data(:,1) == 3, :);
    imu3.t = imu3_data(:,2)/1000;
    imu3.accX.meas = imu3_data(:,3)*9.81;
    imu3.accY.meas = imu3_data(:,4)*9.81;
    imu3.accZ.meas = imu3_data(:,5)*9.81;
    imu3.gyroX.meas = deg2rad(imu3_data(:,6));
    imu3.gyroY.meas = deg2rad(imu3_data(:,7));
    imu3.gyroZ.meas = deg2rad(imu3_data(:,8));
    imu3.magX.meas = imu3_data(:,9)*0.01;
    imu3.magY.meas = imu3_data(:,10)*0.01;
    imu3.magZ.meas = imu3_data(:,11)*0.01;
    
    
    % get IMU 4 data
    imu4_data = data(data(:,1) == 4, :);
    imu4.t = imu4_data(:,2)/1000;
    imu4.accX.meas = imu4_data(:,3)*9.81;
    imu4.accY.meas = imu4_data(:,4)*9.81;
    imu4.accZ.meas = imu4_data(:,5)*9.81;
    imu4.gyroX.meas = deg2rad(imu4_data(:,6));
    imu4.gyroY.meas = deg2rad(imu4_data(:,7));
    imu4.gyroZ.meas = deg2rad(imu4_data(:,8));
    imu4.magX.meas = imu4_data(:,9)*0.01;
    imu4.magY.meas = imu4_data(:,10)*0.01;
    imu4.magZ.meas = imu4_data(:,11)*0.01;


end

