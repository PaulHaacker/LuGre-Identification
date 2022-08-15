% fit static parameters from dynamic experiment

%% run simulation
close all

x_0 = [0 0 0]';
tspan = [0 10];

% control mode, controller gains, control target and system parameter are
% hard coded within the fcn.
F_RC = 5;
F_RS = 20;
v_s = .07;
sigma_0 = 133.3e3;

sigma_1 = 600;
sigma_2 = 35;

parameters_2 = [F_RC,F_RS,v_s,sigma_0];
parameters_1 = [sigma_0,sigma_1,sigma_2];

parameter.model.fric.para2 = parameters_2;
parameter.model.fric.para1 = parameters_1;
parameter.model.m = 5;

parameter.controller.mode = 'FF_swelling';

[t,x] = friction_lugre_sim(tspan,x_0,parameter);

% reconstruct the input signal
u_control = zeros(size(t));
for k = 1:length(t)
    u_control(k) = F_control(t(k),x(k,:),parameter);
end

%% plots

figure
plot(t,x(:,2))
ylabel('velocity')
xlabel('time')
grid on

% figure
% plot(x(:,2),u_control)
% ylabel('velocity')
% xlabel('time')