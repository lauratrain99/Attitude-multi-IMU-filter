function [Jcost] = LQR(time, true_sol, exp_sol)
% LQR computes the linear quadratic regulator of the error
% between true_sol and exp_sol
%
% INPUT:
%         time, vector of time of the domain
%     true_sol, analytical solution
%      exp_sol, KF solution
%
% OUTPUT:
%        Jcost, cost functional value
%
%%
    Jcost = trapz(time, abs(true_sol - exp_sol).^2);
    
end

