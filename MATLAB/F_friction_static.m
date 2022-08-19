function F_friction_static = F_friction_static(v,theta)
% static friction force: returns static friction force when ddot(q) = 0 and
% dot(q) = v = const for the LuGre model.
%   there are 4 static friction paramteters theta = (tau_c, tau_s, v_s,
%   sigma_2).

% tau_c = theta.tau_c;
% tau_s = theta.tau_s;
% v_s = theta.v_s;
% sigma_2 = theta.sigma_2;

% lsqnonlin can only handle the parameters to be matrices
tau_c = theta(1);
tau_s = theta(2);
v_s = theta(3);
sigma_2 = theta(4);

F_friction_static = sigma_2*v + (tau_c + (tau_s - tau_c)*exp(-(v/v_s).^2))...
    .*sign(v);
end

