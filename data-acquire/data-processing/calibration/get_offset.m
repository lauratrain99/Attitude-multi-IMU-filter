function [offset, std] = get_offset(meas, expected_value)
    offset=mean(meas) - expected_value;

    noisemeas=meas-mean(offset);
    var_meas=var(noisemeas);

    % Find the standard deviation of total sensor measurements 
    std = sqrt(var_meas);
end

