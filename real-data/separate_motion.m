function [mov] = separate_motion(start_pos, end_pos, imu1, imu2, imu3, imu4)
    mov.imu1 = imu1;
    mov.imu1.t = imu1.t(start_pos:end_pos);
    mov.imu1.wb = imu1.wb(start_pos:end_pos,:);
    mov.imu1.fb = imu1.fb(start_pos:end_pos,:);

    mov.imu2 = imu2;
    mov.imu2.t = imu2.t(start_pos:end_pos);
    mov.imu2.wb = imu2.wb(start_pos:end_pos,:);
    mov.imu2.fb = imu2.fb(start_pos:end_pos,:);

    mov.imu3 = imu3;
    mov.imu3.t = imu3.t(start_pos:end_pos);
    mov.imu3.wb = imu3.wb(start_pos:end_pos,:);
    mov.imu3.fb = imu3.fb(start_pos:end_pos,:);

    mov.imu4 = imu4;
    mov.imu4.t = imu4.t(start_pos:end_pos);
    mov.imu4.wb = imu4.wb(start_pos:end_pos,:);
    mov.imu4.fb = imu4.fb(start_pos:end_pos,:);
end

