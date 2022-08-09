% test lugre friction - simple 1D body with friction
close all

x_0 = [0 0 0]';
tspan = [0 10];

[t,x] = friction_lugre_fcn(tspan,x_0);

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
    u_control(k) = F_an(t(k),x(k,:));
end

v = x(:,2);
z = x(:,3);
z_dot = v-abs(v)./g_fric(v).*z;
F_R_num = F_R(v,z,z_dot);
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

