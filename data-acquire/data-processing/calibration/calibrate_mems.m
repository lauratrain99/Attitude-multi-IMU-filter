function [fs, bias_stab, std, offset, T, sigma] = calibrate_mems(meas, time, expected_value)
% calibrate_mems computes the calibration parameters for
% accelerometer or gyroscope

    %% Data analysis
    pts=10000;

    offset=mean(meas) - expected_value;

    noisemeas=meas-mean(offset);
    var_meas=var(noisemeas);

    % Find the standard deviation of total sensor measurements 
    std = sqrt(var_meas);
    
    % Compute sampling frequency
    fs = 1/median(diff(time));

    % Compute Allan Variance From Data
    [T,sigma] = allan(meas,fs,pts);

    % Compute PSD from Data
    [Pxx,f] = computePowerSpectralDensities( meas,fs );

    %% Compute critical system parameters

    tau_low_l =10^(-3);
    tau_high_l =10^2;
    tau_low_r = 10^2;
    tau_high_r =10^3;

    count = 1;
    for i = 1:length(T)
        if (T(i) >= tau_low_l && T(i) <= tau_high_l)
            tau(count) = T(i); 
            ADev(count) = sigma(i); 
            count = count + 1;
        end
    end

    % Use bisection method to compute N (the mean square value of the angle
    % random walk noise)

    higher = 1;
    lower = 1e-10;
    for i = 1:100
        N = (higher+lower)/2;
        error = sum(N./sqrt(tau) - ADev);
        if error < 0 
            lower = N;
        else 
            higher = N;
        end
    end

    % Find Allan Deviation Along Rate Random Walk Portion of Curve
    % tau_low = input('Input right side lower time window bound: ');
    % tau_high = input('Input right side higher time window bound: ');

    count = 1;
    for i = 1:length(T)
        if (T(i) >= tau_low_r && T(i) <= tau_high_r)
            tau_right(count) = T(i); 
            ADev_right(count) = sigma(i);
            count = count + 1;
        end
    end

    % Use bisection method to compute K (the mean square value of the rate
    % random walk noise)
    higher = 1;
    lower = 1e-10;
    for i = 1:100
        K = (higher+lower)/2;
        error = sum(K*sqrt(tau_right/3) - ADev_right);
        if error < 0
            lower = K;
        else
            higher = K;
        end
    end

    % Find the Time Window at Minimum of Allan Variance
    index = find(sigma == min(sigma));
    Tc = T(index);

    % Find B (the mean square value of the bias instability)
    bias_stab = min(sigma)/sqrt(2*log(2)/pi);


end

