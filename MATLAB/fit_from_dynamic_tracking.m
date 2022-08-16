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

[t,x,u_control] = friction_lugre_sim(tspan,x_0,parameter);

% read velocity
v = x(:,2);

% exactly extract friction force. in the real application, we even need to
% find friction force by subtracting inertia term from input.
z = x(:,3);
z_dot = v-abs(v)./g_fric(v,parameter).*z;
F_R_num = F_R(v,z,z_dot,parameter);

%% identification
% true parameter
theta_true = [parameter.model.fric.para2(1:3),parameter.model.fric.para1(3)];

% initial parameter for optimization
theta_0 = theta_true*5;

% optimization
[theta_ident,resnorm] = StaticFrictionCurveFitting(theta_0,v,...
    F_R_num);

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

N_plot = 500;
v_plot = linspace(0,1,N_plot);

figure
plot(v,F_R_num,'.','Color',.9*ones(1,3))
grid on
% ylim([0 80])
xlabel('velocity v')
ylabel('static friction')
% ylim([0 50])
hold on
plot(v_plot,F_friction_static(v_plot,theta_ident))
plot(v_plot,F_friction_static(v_plot,theta_true))
annotation('textbox',[.15,.6,.3,.3],'String',['\theta_{true}=(',...
    num2str(theta_true),').\newline \theta_{ident}=(',...
    num2str(theta_ident),').'])
legend('data','identified','true')
title('static friction curve fitting')