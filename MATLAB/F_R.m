function F_R = F_R(v,z,z_dot,parameter)
% reibkraft

parameter = parameter.model.fric.para1;

sigma_0 = parameter(1);
sigma_1 = parameter(2);
sigma_2 = parameter(3);

F_R = sigma_0*z + sigma_1*z_dot + sigma_2*v;
end