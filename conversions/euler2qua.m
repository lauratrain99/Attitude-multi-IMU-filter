function qua = euler2qua(euler)
% euler2qua converts from Euler angles to quaternions
% 
% INPUT:
%           euler, 3x1 Euler angles [rad, rad, rad]
%
% OUTPUT:
%             qua, 4x1 quaternion
%
%%
    % Rearrange vector for 3-2-1 rotation sequence
    euler = [euler(3) euler(2) euler(1)];

    c_eul = cos( euler/2 );
    s_eul = sin( euler/2 );

    % 3-2-1 rotation sequence
    q = [c_eul(1)*c_eul(2)*c_eul(3) + s_eul(1)*s_eul(2).*s_eul(3), ...
         c_eul(1)*c_eul(2)*s_eul(3) - s_eul(1)*s_eul(2).*c_eul(3), ...
         c_eul(1)*s_eul(2)*c_eul(3) + s_eul(1)*c_eul(2).*s_eul(3), ...
         s_eul(1)*c_eul(2)*c_eul(3) - c_eul(1)*s_eul(2).*s_eul(3)];

    qua = [q(2) q(3) q(4) q(1)]';

end
