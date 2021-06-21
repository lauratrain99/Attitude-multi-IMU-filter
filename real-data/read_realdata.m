function [imu1, imu2, imu3, imu4] = read_realdata(data)
    
    % get IMU 1 data
    imu1_data = data(data(:,1) == 1, :);
    imu1.t = imu1_data(:,2)/1000;
    imu1.fb(:,1) = imu1_data(:,3)*9.81;
    imu1.fb(:,2) = imu1_data(:,4)*9.81;
    imu1.fb(:,3) = imu1_data(:,5)*9.81;
    imu1.wb(:,1) = deg2rad(imu1_data(:,6));
    imu1.wb(:,2) = deg2rad(imu1_data(:,7));
    imu1.wb(:,3) = deg2rad(imu1_data(:,8));
    imu1.mb(:,1) = imu1_data(:,9)*0.01;
    imu1.mb(:,2) = imu1_data(:,10)*0.01;
    imu1.mb(:,3) = imu1_data(:,11)*0.01;

    % get IMU 2 data
    imu2_data = data(data(:,1) == 2, :);
    imu2.t = imu2_data(:,2)/1000;
    imu2.fb(:,1) = imu2_data(:,3)*9.81;
    imu2.fb(:,2) = imu2_data(:,4)*9.81;
    imu2.fb(:,3) = imu2_data(:,5)*9.81;
    imu2.wb(:,1) = deg2rad(imu2_data(:,6));
    imu2.wb(:,2) = deg2rad(imu2_data(:,7));
    imu2.wb(:,3) = deg2rad(imu2_data(:,8));
    imu2.mb(:,1) = imu2_data(:,9)*0.01;
    imu2.mb(:,2) = imu2_data(:,10)*0.01;
    imu2.mb(:,3) = imu2_data(:,11)*0.01;
    
    % get IMU 3 data
    imu3_data = data(data(:,1) == 3, :);
    imu3.t = imu3_data(:,2)/1000;
    imu3.fb(:,1) = imu3_data(:,3)*9.81;
    imu3.fb(:,2) = imu3_data(:,4)*9.81;
    imu3.fb(:,3) = imu3_data(:,5)*9.81;
    imu3.wb(:,1) = deg2rad(imu3_data(:,6));
    imu3.wb(:,2) = deg2rad(imu3_data(:,7));
    imu3.wb(:,3) = deg2rad(imu3_data(:,8));
    imu3.mb(:,1) = imu3_data(:,9)*0.01;
    imu3.mb(:,2) = imu3_data(:,10)*0.01;
    imu3.mb(:,3) = imu3_data(:,11)*0.01;
    
    
    % get IMU 4 data
    imu4_data = data(data(:,1) == 4, :);
    imu4.t = imu4_data(:,2)/1000;
    imu4.fb(:,1) = imu4_data(:,3)*9.81;
    imu4.fb(:,2) = imu4_data(:,4)*9.81;
    imu4.fb(:,3) = imu4_data(:,5)*9.81;
    imu4.wb(:,1) = deg2rad(imu4_data(:,6));
    imu4.wb(:,2) = deg2rad(imu4_data(:,7));
    imu4.wb(:,3) = deg2rad(imu4_data(:,8));
    imu4.mb(:,1) = imu4_data(:,9)*0.01;
    imu4.mb(:,2) = imu4_data(:,10)*0.01;
    imu4.mb(:,3) = imu4_data(:,11)*0.01;


end



