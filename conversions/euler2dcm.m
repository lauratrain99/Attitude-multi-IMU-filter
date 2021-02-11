function DCMnb = euler2dcm(euler)
% euler2dcm: converts from Euler angles to DCM nav-to-body.
% 
% INPUT
%   euler: 3x1 Euler angles [roll pitch yaw] (rad, rad, rad).
%
% OUTPUT
%   DCMnb: 3x3 nav-to-body DCM.
%
%%
  roll = euler(1); pitch = euler(2); yaw = euler(3);

  R1 = [cos(yaw)  sin(yaw) 0; ...
        -sin(yaw) cos(yaw) 0; ...
         0     0   1];
     
  R2 = [cos(pitch)  0  -sin(pitch); ...
          0   1     0 ; ...
        sin(pitch)  0   cos(pitch)];
    
  R3 = [1   0    0;   ...
        0  cos(roll) sin(roll); ...
        0 -sin(roll) cos(roll)];  
 
  DCMnb = R3 * R2 * R1;

end
