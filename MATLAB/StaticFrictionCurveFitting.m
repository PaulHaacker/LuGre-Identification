function [theta_ident,resnorm] = StaticFrictionCurveFitting(theta_0,v_data,...
    f_data)
% reshape data into column vec
if isrow(v_data)
    v_data = v_data';
end
if isrow(f_data)
    f_data = f_data';
end

% parameter bounds
theta_lb = zeros(4,1);
theta_ub = inf*ones(4,1);

% optimization
[theta_ident,resnorm] = lsqnonlin(@(theta)diff_fric(theta,v_data,...
    f_data),theta_0, ...
    theta_lb, theta_ub);
end

function diff = diff_fric(theta,v_data,f_data)
% returns the difference of measured (or true) friction and computed one
% via velocities v_data and parameters theta
% inputs must be:
% theta - 4x1 matrix
% v_data - Nx1 matrix
% f_data - Nx1 matrix

diff = f_data - F_friction_static(v_data,theta);

end