% test lugre friction - simple 1D body with friction
close all

x_0 = [0 0 0]';
opts = odeset('MaxStep',1e-3);
[t,x] = ode23tb(@ode,[0 1],x_0,opts);

plot(t,x(:,2),'.-')
legend('velocity')
grid on
figure
plot(t,x(:,3),'.-')
legend('LuGre displacement')
grid on

% plot der reibkraft

v = x(:,2);
z = x(:,3);
z_dot = v-abs(v)./g_fric(v).*z;
F_R_num = F_R(v,z,z_dot);
figure
plot(v,F_R_num,'.-')
xlabel('velocity v')
ylabel('friction F_R')
grid on

% plot of step size
figure
plot(t(2:end)-t(1:end-1))
ylabel('step size')
xlabel('step number')

function dx = ode(t,x)

% first order ODE, state is body position y & velocity y_dot and bristle
% deplacement z. x = [ y ; y_dot ; z ]
dx = zeros(size(x));

m = 5;

v = x(2);
z = x(3);

dx(1) = v;
dx(3) = v-abs(v)/g_fric(v)*z;

z_dot = dx(3);
dx(2) = (F_an(t)-F_R(v,z,z_dot))/m;

end

function F_an = F_an(t)
% antreibende kraft
if t<inf
    F_an = 100*t;
else
    F_an = 0;
end
end

function F_R = F_R(v,z,z_dot)
% reibkraft

[parameter,~] = paramter_fcn();

sigma_0 = parameter(1);
sigma_1 = parameter(2);
sigma_2 = parameter(3);

F_R = sigma_0*z + sigma_1*z_dot + sigma_2*v;
end

function g_fric = g_fric(v)

[~,parameter] = paramter_fcn();

F_RC = parameter(1);
F_RS = parameter(2);
v_s = parameter(3);
sigma_0 = parameter(4);
g_fric = (F_RC + (F_RS-F_RC)*exp(-(v/v_s).^2))/sigma_0;
end

function [parameters_1,parameters_2] = paramter_fcn()
% F_RC = 1;
% F_RS = 1.5;
% v_s = 1e-3;
% sigma_0 = 1e5;
% 
% sigma_1 = sqrt(sigma_0);
% sigma_2 = .4;

F_RC = 5;
F_RS = 20;
v_s = .07;
sigma_0 = 133.3e3;

sigma_1 = 600;
sigma_2 = 35;

parameters_2 = [F_RC,F_RS,v_s,sigma_0];
parameters_1 = [sigma_0,sigma_1,sigma_2];

end
