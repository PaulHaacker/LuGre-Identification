% fit static parameters from dynamic experiment

%% run simulation
close all

x_0 = [0 0 0]';
T_S = .001;
tspan = 0:T_S:50; 

% controller gains, control target are
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

%% identification, iv method

iv_opt = iv4Options('Focus','simulation'); % gets an error when included
data_presliding = iddata(q,u_control,T_S);
sys_iv = iv4(data_presliding, [2 1 0]);

b0 = sys_iv.B;
a0 = sys_iv.A(3);
a1 = sys_iv.A(2);

% extract known viscous friction - in real world application, this needs to
% be identified
tau_v = sigma_2;

% find original parameters
parameter_ident_iv.J = T_S^2/b0;
parameter_ident_iv.sigma_1 = parameter_ident_iv.J/T_S*(a1+2)-tau_v;
parameter_ident_iv.sigma_0 = (parameter_ident_iv.J*(a0-1)+ ...
    T_S*(parameter_ident_iv.sigma_1+tau_v))/T_S^2;

%% identification, arx method

% arx_opt = iv4Options('Focus','simulation');
data_presliding = iddata(q,u_control,T_S);
sys_arx = arx(data_presliding, [2 1 0]);

b0 = sys_arx.B;
a0 = sys_arx.A(3);
a1 = sys_arx.A(2);

% extract known viscous friction - in real world application, this needs to
% be identified
tau_v = sigma_2;

% find original parameters
parameter_ident_arx.J = T_S^2/b0;
parameter_ident_arx.sigma_1 = parameter_ident_arx.J/T_S*(a1+2)-tau_v;
parameter_ident_arx.sigma_0 = (parameter_ident_arx.J*(a0-1)+ ...
    T_S*(parameter_ident_arx.sigma_1+tau_v))/T_S^2;


%% plots
% % find cost (residual) of true parameters
% resnorm_ident = sum(diff_fric(theta_ident,v,F_R_num).^2); % should equal to resnorm above
% resnorm_true = sum(diff_fric(theta_true,v,F_R_num).^2);
% 

% plot velocity signal
figure
plot(t,v,'.-')
ylabel('velocity')
xlabel('time')
grid on
% plot position signal
figure
plot(t,q,'.-')
ylabel('position')
xlabel('time')
grid on
% annotation('textbox',[.15,.6,.3,.3],'String',['\theta_{true}=(J \sigma_0 \sigma_1)=(',...
%     num2str([parameter.model.m,parameter.model.fric.para1(1:2)]),...
%     ').\newline \theta_{ident}=(',...
%     num2str([parameter_ident_iv.J,parameter_ident_iv.sigma_0,...
%     parameter_ident_iv.sigma_1]),').'])

disp('______________________________________________________________________')
disp('Results of iv4')
disp(['\theta_{true}=(J \sigma_0 \sigma_1)=(',...
    num2str([parameter.model.m,parameter.model.fric.para1(1:2)]),...
    ').'])
disp(['\theta_{ident}=(',...
    num2str([parameter_ident_iv.J,parameter_ident_iv.sigma_0,...
    parameter_ident_iv.sigma_1]),').'])
disp('______________________________________________________________________')
disp('Results of arx')
disp(['\theta_{true}=(J \sigma_0 \sigma_1)=(',...
    num2str([parameter.model.m,parameter.model.fric.para1(1:2)]),...
    ').'])
disp(['\theta_{ident}=(',...
    num2str([parameter_ident_arx.J,parameter_ident_arx.sigma_0,...
    parameter_ident_arx.sigma_1]),').'])


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