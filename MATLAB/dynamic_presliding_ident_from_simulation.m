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

parameter.controller.mode = 'FF_presliding';

[t,x,u_control] = friction_lugre_sim(tspan,x_0,parameter);

% read velocity
v = x(:,2);
q = x(:,1);

%% identification


%% plots
% % find cost (residual) of true parameters
% resnorm_ident = sum(diff_fric(theta_ident,v,F_R_num).^2); % should equal to resnorm above
% resnorm_true = sum(diff_fric(theta_true,v,F_R_num).^2);
% 

% plot velocity signal
figure
plot(t,v)
ylabel('velocity')
xlabel('time')
grid on
% plot position signal
figure
plot(t,q)
ylabel('position')
xlabel('time')
grid on


% 
% % figure
% % plot(x(:,2),u_control)
% % ylabel('velocity')
% % xlabel('time')
% 
% % plot static friction curve
% N_plot = 500;
% v_plot = linspace(-1,1,N_plot);
% 
% figure
% plot(v,F_R_num,'.','Color',.9*ones(1,3))
% grid on
% % ylim([0 80])
% xlabel('velocity v')
% ylabel('static friction')
% % ylim([0 50])
% hold on
% plot(v_plot,F_friction_static(v_plot,theta_ident))
% plot(v_plot,F_friction_static(v_plot,theta_true))
% annotation('textbox',[.15,.6,.3,.3],'String',['\theta_{true}=(',...
%     num2str(theta_true),').\newline \theta_{ident}=(',...
%     num2str(theta_ident),').'])
% legend('data','identified','true')
% title('static friction curve fitting')