function [imu] = calibrate_imu(imu, acc_desired)
    [imu.gyroX.fs, imu.gyroX.bias_stab, imu.gyroX.std, imu.gyroX.offset, imu.gyroX.T, imu.gyroX.sigma] = calibrate_mems(imu.gyroX.meas, imu.t, 0);
    [imu.gyroY.fs, imu.gyroY.bias_stab, imu.gyroY.std, imu.gyroY.offset, imu.gyroY.T, imu.gyroY.sigma] = calibrate_mems(imu.gyroY.meas, imu.t, 0);
    [imu.gyroZ.fs, imu.gyroZ.bias_stab, imu.gyroZ.std, imu.gyroZ.offset, imu.gyroZ.T, imu.gyroZ.sigma] = calibrate_mems(imu.gyroZ.meas, imu.t, 0);
    
    [imu.accX.fs, imu.accX.bias_stab, imu.accX.std, imu.accX.offset, imu.accX.T, imu.accX.sigma] = calibrate_mems(imu.accX.meas, imu.t, acc_desired.x);
    [imu.accY.fs, imu.accY.bias_stab, imu.accY.std, imu.accY.offset, imu.accY.T, imu.accY.sigma] = calibrate_mems(imu.accY.meas, imu.t, acc_desired.y);
    [imu.accZ.fs, imu.accZ.bias_stab, imu.accZ.std, imu.accZ.offset, imu.accZ.T, imu.accZ.sigma] = calibrate_mems(imu.accZ.meas, imu.t, acc_desired.z);

end

