function [t,x,u_control] = friction_lugre_sim(tspan,x_0,parameter)
%friction_lugre_fcn - simulate the body with friction

% lugre friction - simple 1D body with friction
% close all

% F_RC = 5;
% F_RS = 20;
% v_s = .07;
% sigma_0 = 133.3e3;
% 
% sigma_1 = 600;
% sigma_2 = 35;
% 
% parameters_2 = [F_RC,F_RS,v_s,sigma_0];
% parameters_1 = [sigma_0,sigma_1,sigma_2];
% 
% parameter.model.fric.para2 = parameters_2;
% parameter.model.fric.para1 = parameters_1;
% parameter.model.m = 5;
% 
% parameter.controller.mode = 'FB_PI_velocity_control_FF_breakaway_Schedule_setpoint';

% x_0 = [0 0 0]';
opts = odeset('MaxStep',1e-3);
[t,x] = ode23tb(@(t,x)ode(t,x,parameter),tspan,x_0,opts);

% reconstruct the input signal
u_control = zeros(size(t));
for k = 1:length(t)
    u_control(k) = F_control(t(k),x(k,:),parameter);
end

% all plots are now outside the fcn

% plot(t,x(:,2),'.-')
% legend('velocity')
% xlabel('time t')
% grid on
% movegui('northwest')
% figure
% plot(t,x(:,3),'.-')
% legend('LuGre displacement')
% grid on
% movegui('northeast')
% 
% % plot der reibkraft und control input
% u_control = zeros(size(t));
% for k = 1:length(t)
%     u_control(k) = F_control(t(k),x(k,:));
% end
% 
% v = x(:,2);
% z = x(:,3);
% z_dot = v-abs(v)./g_fric(v).*z;
% F_R_num = F_R(v,z,z_dot);
% figure
% plot(v,F_R_num,'.-')
% hold on
% plot(v,u_control,'.-')
% xlabel('velocity v')
% legend('friction F_R','control input u')
% grid on
% movegui('southeast')
% 
% % plot of step size
% figure
% plot(t(2:end)-t(1:end-1))
% ylabel('step size')
% xlabel('step number')
% movegui('southwest')

end

function dx = ode(t,x,parameter)

% first order ODE, state is body position y & velocity y_dot and bristle
% deplacement z. x = [ y ; y_dot ; z ]
dx = zeros(size(x));

m = parameter.model.m;

v = x(2);
z = x(3);

dx(1) = v;
dx(3) = v-abs(v)/g_fric(v,parameter)*z;

z_dot = dx(3);
dx(2) = (F_control(t,x,parameter)-F_R(v,z,z_dot,parameter))/m;

end

% function [parameters_1,parameters_2] = paramter_fcn()
% % F_RC = 1;
% % F_RS = 1.5;
% % v_s = 1e-3;
% % sigma_0 = 1e5;
% % 
% % sigma_1 = sqrt(sigma_0);
% % sigma_2 = .4;
% 
% F_RC = 5;
% F_RS = 20;
% v_s = .07;
% sigma_0 = 133.3e3;
% 
% sigma_1 = 600;
% sigma_2 = 35;
% 
% parameters_2 = [F_RC,F_RS,v_s,sigma_0];
% parameters_1 = [sigma_0,sigma_1,sigma_2];
% 
% end


