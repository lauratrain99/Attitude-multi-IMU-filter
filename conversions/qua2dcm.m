function DCMbn = qua2dcm(qua)
% qua2dcm: transforms quaternion to body-to-nav DCM.
%
% INPUT
%   qua: 4x1 quaternion.
%
% OUTPUT
%   DCMbn: 3x3 body-to-nav DCM.
%
%%

% Quaternion format from Crassidis' book.
a = qua(4); b = qua(1); c = qua(2); d = qua(3);

DCMbn(1,1) = a*a + b*b - c*c - d*d;
DCMbn(1,2) = 2*(b*c - a*d);
DCMbn(1,3) = 2*(a*c + b*d);
DCMbn(2,1) = 2*(a*d + b*c);
DCMbn(2,2) = a*a - b*b + c*c - d*d;
DCMbn(2,3) = 2*(c*d - a*b);
DCMbn(3,1) = 2*(b*d - a*c);
DCMbn(3,2) = 2*(c*d + a*b);
DCMbn(3,3) = a*a - b*b - c*c + d*d;

end
