a = [0.22,0,0.17];
q01 = [-1,1,0,0]/norm([-1,1,0,0]);
arot = quatmultiply(quatmultiply(quatinv(q01),[0,a]),q01)
quatmultiply(quatinv(q01),[0,a])

qmod = [q01(2), q01(3), q01(4), q01(1)];
arot_prime = qua2dcm(qmod)'*a'
