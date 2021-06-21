function euler = dcm2euler(DCMbn)
% dcm2euler converts from body-to-nav DCM to Euler angles
%
% INPUT:
%           DCMbn, 3x3 body-to-nav DCM.
%
% OUTPUT:
%           euler:, 3x1 Euler angles [rad, rad, rad]
%
%%
    roll   =  atan2( DCMbn(3,2) / DCMbn(3,3) ); 
    pitch = - asin( DCMbn(3,1) );                
    yaw   =  atan2( DCMbn(2,1), DCMbn(1,1) ); 

    euler = [roll pitch yaw];

end
