function qua_n = qua_update(qua, wb_n, dt)
% qua_update: updates quaternions.
%
% INPUT:
%   qua,	4x1 quaternion.
%   wb_n,	3x1 incremental turn rates in body-frame (rad/s).
%   dt,     1x1 IMU sampling interval (s).
%
% OUTPUT:
%   qua_n,      4x1 updated quaternion.

%% page 39 Alberto Nieto book

w = 1/2 * wb_n * dt;
wnorm = norm(w);

if wnorm == 0
    
    cos_const = 1;
    sin_const = 1;
else
    
    cos_const=cos(wnorm);
    sin_const=sin(wnorm)/wnorm;
end
    
W=[  0  -w(1) -w(2) -w(3);
   w(1)    0  -w(3)  w(2);
   w(2)  w(3)    0  -w(1);
   w(3) -w(2)  w(1)    0];
I = eye(4);
qua_inv = [qua(4); qua(1); qua(2); qua(3)];
qua_inv_n = (I*cos_const + W*sin_const) * qua_inv;
qua_n = [qua_inv_n(2); qua_inv_n(3); qua_inv_n(4); qua_inv_n(1)];

% wnorm = norm(wb_n);
% 
% if wnorm == 0
%     
%     qua_n = qua;
% else
%     
%     co=cos(0.5*wnorm*dt);
%     si=sin(0.5*wnorm*dt);
%     
%     n1 = wb_n(1)/wnorm;
%     n2 = wb_n(2)/wnorm;
%     n3 = wb_n(3)/wnorm;
%     
%     qw1 = n1*si;
%     qw2 = n2*si;
%     qw3 = n3*si;
%     qw4 = co;
%     
%     Om=[ qw4  qw3 -qw2 qw1;
%         -qw3  qw4  qw1 qw2;
%          qw2 -qw1  qw4 qw3;
%         -qw1 -qw2 -qw3 qw4];
%     
%     qua_n = Om * qua;
% end

end
