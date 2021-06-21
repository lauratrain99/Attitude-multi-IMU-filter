function mov = get_sigmas_arch2(mov)
    mov.imu1.plusSigma.alpha(:,1) =  mov.nav1.Pp(:,1);
    mov.imu1.minusSigma.alpha(:,1) = - mov.nav1.Pp(:,1);

    mov.imu1.plusSigma.alpha(:,2) =  mov.nav1.Pp(:,6*1 + 2);
    mov.imu1.minusSigma.alpha(:,2) = - mov.nav1.Pp(:,6*1 + 2);

    mov.imu1.plusSigma.alpha(:,3) =  mov.nav1.Pp(:,6*2 + 3);
    mov.imu1.minusSigma.alpha(:,3) =  - mov.nav1.Pp(:,6*2 + 3);

    mov.imu2.plusSigma.alpha(:,1) = mov.nav2.Pp(:,1);
    mov.imu2.minusSigma.alpha(:,1) =  - mov.nav2.Pp(:,1);

    mov.imu2.plusSigma.alpha(:,2) =  mov.nav2.Pp(:,6*1 + 2);
    mov.imu2.minusSigma.alpha(:,2) = - mov.nav2.Pp(:,6*1 + 2);

    mov.imu2.plusSigma.alpha(:,3) =  mov.nav2.Pp(:,6*2 + 3);
    mov.imu2.minusSigma.alpha(:,3) =  - mov.nav2.Pp(:,6*2 + 3);

    mov.imu3.plusSigma.alpha(:,1) = mov.nav3.Pp(:,1);
    mov.imu3.minusSigma.alpha(:,1) =  - mov.nav3.Pp(:,1);

    mov.imu3.plusSigma.alpha(:,2) =  mov.nav3.Pp(:,6*1 + 2);
    mov.imu3.minusSigma.alpha(:,2) = - mov.nav3.Pp(:,6*1 + 2);

    mov.imu3.plusSigma.alpha(:,3) =  mov.nav3.Pp(:,6*2 + 3);
    mov.imu3.minusSigma.alpha(:,3) =  - mov.nav3.Pp(:,6*2 + 3);

    mov.imu4.plusSigma.alpha(:,1) = mov.nav4.Pp(:,1);
    mov.imu4.minusSigma.alpha(:,1) =  - mov.nav4.Pp(:,1);

    mov.imu4.plusSigma.alpha(:,2) =  mov.nav4.Pp(:,6*1 + 2);
    mov.imu4.minusSigma.alpha(:,2) = - mov.nav4.Pp(:,6*1 + 2);

    mov.imu4.plusSigma.alpha(:,3) =  mov.nav4.Pp(:,6*2 + 3);
    mov.imu4.minusSigma.alpha(:,3) =  - mov.nav4.Pp(:,6*2 + 3);
end
