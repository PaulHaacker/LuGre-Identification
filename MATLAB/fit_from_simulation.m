% static fitting from simulation


%% Simulate the system in Matlab
% test lugre friction - simple 1D body with friction
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

parameter.controller.mode = 'FB_PI_velocity_control_FF_breakaway_Schedule_setpoint';

[t,x] = friction_lugre_sim(tspan,x_0,parameter);

% reconstruct the input signal
u_control = zeros(size(t));
for k = 1:length(t)
    u_control(k) = F_control(t(k),x(k,:),parameter);
end

% add noise to the signal
% v_noise = x(:,2)+data_noise(x(:,2));
 v_noise = x(:,2); % no noise
 
%% Process the data

[u_points,v_points] = get_static_data(t,v_noise,u_control);


%% curve fitting of the static friction

% true parameter
theta_true = [parameter.model.fric.para2(1:3),parameter.model.fric.para1(3)];

% initial parameter for optimization
theta_0 = theta_true*5;

% optimization
[theta_ident,resnorm] = StaticFrictionCurveFitting(theta_0,v_points,...
    u_points);

%% plots of the result

N_plot = 500;
v_plot = linspace(0,1,N_plot);

figure
plot(v_points,u_points,'.')
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
