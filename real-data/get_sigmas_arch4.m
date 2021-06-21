function mov = get_sigmas_arch4(mov)
    mov.imu1.plusSigma.alpha(:,1) =  mov.navCM.Pp(:,1);
    mov.imu1.minusSigma.alpha(:,1) = -mov.navCM.Pp(:,1);

    mov.imu1.plusSigma.alpha(:,2) =  mov.navCM.Pp(:,24*1 + 2);
    mov.imu1.minusSigma.alpha(:,2) = - mov.navCM.Pp(:,24*1 + 2);

    mov.imu1.plusSigma.alpha(:,3) =  mov.navCM.Pp(:,24*2 + 3);
    mov.imu1.minusSigma.alpha(:,3) =  - mov.navCM.Pp(:,24*2 + 3);

    mov.imu2.plusSigma.alpha(:,1) =  mov.navCM.Pp(:,24*6 + 7);
    mov.imu2.minusSigma.alpha(:,1) =  - mov.navCM.Pp(:,24*6 + 7);

    mov.imu2.plusSigma.alpha(:,2) = mov.navCM.Pp(:,24*7 + 8);
    mov.imu2.minusSigma.alpha(:,2) = - mov.navCM.Pp(:,24*7 + 8);

    mov.imu2.plusSigma.alpha(:,3) =  mov.navCM.Pp(:,24*8 + 9);
    mov.imu2.minusSigma.alpha(:,3) = - mov.navCM.Pp(:,24*8 + 9);

    mov.imu3.plusSigma.alpha(:,1) =  mov.navCM.Pp(:,24*12 + 13);
    mov.imu3.minusSigma.alpha(:,1) = - mov.navCM.Pp(:,24*12 + 13);

    mov.imu3.plusSigma.alpha(:,2) =  mov.navCM.Pp(:,24*13 + 14);
    mov.imu3.minusSigma.alpha(:,2) = - mov.navCM.Pp(:,24*13 + 14);

    mov.imu3.plusSigma.alpha(:,3) =  mov.navCM.Pp(:,24*14 + 15);
    mov.imu3.minusSigma.alpha(:,3) = - mov.navCM.Pp(:,24*14 + 15);

    mov.imu4.plusSigma.alpha(:,1) =  mov.navCM.Pp(:,24*18 + 19);
    mov.imu4.minusSigma.alpha(:,1) =  - mov.navCM.Pp(:,24*18 + 19);

    mov.imu4.plusSigma.alpha(:,2) = mov.navCM.Pp(:,24*19 + 20);
    mov.imu4.minusSigma.alpha(:,2) = - mov.navCM.Pp(:,24*19 + 20);

    mov.imu4.plusSigma.alpha(:,3) = mov.navCM.Pp(:,24*20 + 21);
    mov.imu4.minusSigma.alpha(:,3) = - mov.navCM.Pp(:,24*20 + 21);
end