function g_fric = g_fric(v,parameter)

parameter = parameter.model.fric.para2;

F_RC = parameter(1);
F_RS = parameter(2);
v_s = parameter(3);
sigma_0 = parameter(4);
g_fric = (F_RC + (F_RS-F_RC)*exp(-(v/v_s).^2))/sigma_0;
end