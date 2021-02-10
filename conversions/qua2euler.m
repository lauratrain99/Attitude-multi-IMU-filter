function euler = qua2euler(q)
% qua2euler: transforms quaternion to Euler angles.
%
% INPUT
%   q: 4x1 quaternion.
%
% OUTPUT
%   euler: 3x1 Euler angles [roll pitch yaw] (rad, rad, rad).
%
%%

DCMbn = qua2dcm(q);

roll   = atan2( DCMbn(3,2), DCMbn(3,3) );    % roll
pitch = asin (-DCMbn(3,1) );                % pitch
yaw   = atan2( DCMbn(2,1), DCMbn(1,1) );    % yaw

euler = [roll pitch yaw];

end
