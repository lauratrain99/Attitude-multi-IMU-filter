function euler = qua2euler(qua)
% qua2euler transforms quaternion to Euler angles
%
% INPUT
%             qua, 4x1 quaternion
%
% OUTPUT
%           euler, 3x1 Euler angles [rad, rad, rad]
%
%%
    DCMbn = qua2dcm(qua);

    roll   = atan2( DCMbn(3,2), DCMbn(3,3) );    
    pitch = asin (-DCMbn(3,1) );               
    yaw   = atan2( DCMbn(2,1), DCMbn(1,1) );    

    euler = [roll pitch yaw];

end
