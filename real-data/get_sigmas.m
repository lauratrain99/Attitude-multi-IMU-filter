function mov = get_sigmas(mov)
    mov.plusSigma.alpha(:,1) =  mov.navCM.Pp(:,1);
    mov.minusSigma.alpha(:,1) = -mov.navCM.Pp(:,1);

    mov.plusSigma.alpha(:,2) =  mov.navCM.Pp(:,6*1 + 2);
    mov.minusSigma.alpha(:,2) = - mov.navCM.Pp(:,6*1 + 2);

    mov.plusSigma.alpha(:,3) =  mov.navCM.Pp(:,6*2 + 3);
    mov.minusSigma.alpha(:,3) =  - mov.navCM.Pp(:,6*2 + 3);
end
