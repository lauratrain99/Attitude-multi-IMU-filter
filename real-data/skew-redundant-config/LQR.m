function [Jcost] = LQR(time, true_sol, exp_sol)
    Jcost = trapz(time, abs(true_sol - exp_sol).^2);
end

