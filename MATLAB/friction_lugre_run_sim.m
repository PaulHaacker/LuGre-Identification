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

%% Plot the results

plot(t,x(:,2),'.-')
legend('velocity')
xlabel('time t')
grid on
movegui('northwest')
figure
plot(t,x(:,3),'.-')
legend('LuGre displacement')
grid on
movegui('northeast')

% plot der reibkraft und control input
u_control = zeros(size(t));
for k = 1:length(t)
    u_control(k) = F_control(t(k),x(k,:),parameter);
end

v = x(:,2);
z = x(:,3);
z_dot = v-abs(v)./g_fric(v,parameter).*z;
F_R_num = F_R(v,z,z_dot,parameter);
figure
plot(v,F_R_num,'.-')
hold on
plot(v,u_control,'.-')
xlabel('velocity v')
legend('friction F_R','control input u')
grid on
movegui('southeast')

% plot of step size
figure
plot(t(2:end)-t(1:end-1))
ylabel('step size')
xlabel('step number')
movegui('southwest')

