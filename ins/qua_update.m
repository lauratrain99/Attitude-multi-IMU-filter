function qua_n = qua_update(qua, wb_n, dt)
% qua_update: updates quaternions.
%
% INPUT:
%      qua,	4x1 quaternion.
%     wb_n,	3x1 incremental turn rates in body-frame (rad/s).
%       dt, IMU sampling interval (s).
%
% OUTPUT:
%    qua_n, 4x1 updated quaternion.

%%

w = 1/2 * wb_n * dt;
wnorm = norm(w);

if wnorm == 0
    
    cos_const = 1;
    sin_const = 1;
else
    
    cos_const=cos(wnorm);
    sin_const=sin(wnorm)/wnorm;
end
    
W=[  0  w(3)  -w(2)  w(1);
   -w(3)    0  w(1)  w(2);
    w(2) -w(1)   0   w(3);
   -w(1) -w(2) -w(3)    0];

I = eye(4);
qua_n = (I*cos_const + W*sin_const) * qua;


% w = 1/2 * wb_n * dt;
% wnorm = norm(w);
% 
% if wnorm == 0
%     
%     cos_const = 1;
%     sin_const = 1;
% else
%     
%     cos_const=cos(wnorm);
%     sin_const=sin(wnorm)/wnorm;
% end
%     
% W=[  0  -w(1) -w(2) -w(3);
%    w(1)    0  -w(3)  w(2);
%    w(2)  w(3)    0  -w(1);
%    w(3) -w(2)  w(1)    0];
% I = eye(4);
% qua_inv = [qua(4); qua(1); qua(2); qua(3)];
% qua_inv_n = (I*cos_const + W*sin_const) * qua_inv;
% qua_n = [qua_inv_n(2); qua_inv_n(3); qua_inv_n(4); qua_inv_n(1)];


end
