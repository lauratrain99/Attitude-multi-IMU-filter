function S = skewm(v)
% skewm: forms a skew-symmetric matrix from a 3x1 vector.
%
% INPUT
%     v: 3x1 vector.
%
%	OUTPUT
%	  S: 3x3 skew-symmetric matrix.
%
%%

x = v(1);
y = v(2);
z = v(3);

S = [ 0 -z   y;
      z  0  -x;
     -y  x   0];

end
