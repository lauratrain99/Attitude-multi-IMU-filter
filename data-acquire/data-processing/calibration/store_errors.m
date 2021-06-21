function [imu_error] = store_errors(imu)
%STORE_ERRORS Summary of this function goes here
%   Detailed explanation goes here
    imu_error.accX.bias_stab = imu.accX.bias_stab;
    imu_error.accX.std = imu.accX.std;
    imu_error.accX.offset = imu.accX.offset;

    imu_error.accY.bias_stab = imu.accY.bias_stab;
    imu_error.accY.std = imu.accY.std;
    imu_error.accY.offset = imu.accY.offset;

    imu_error.accZ.bias_stab = imu.accZ.bias_stab;
    imu_error.accZ.std = imu.accZ.std;
    imu_error.accZ.offset = imu.accZ.offset;

    imu_error.gyroX.bias_stab = imu.gyroX.bias_stab;
    imu_error.gyroX.std = imu.gyroX.std;
    imu_error.gyroX.offset = imu.gyroX.offset;

    imu_error.gyroY.bias_stab = imu.gyroY.bias_stab;
    imu_error.gyroY.std = imu.gyroY.std;
    imu_error.gyroY.offset = imu.gyroY.offset;

    imu_error.gyroZ.bias_stab = imu.gyroZ.bias_stab;
    imu_error.gyroZ.std = imu.gyroZ.std;
    imu_error.gyroZ.offset = imu.gyroZ.offset;
end

