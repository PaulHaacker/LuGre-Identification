%% Plot the static friction curve with known, hard coded paramters.

F_RC = 5;
F_RS = 20;
v_s = .07;
sigma_2 = 35;
theta.tau_c = F_RC;
theta.tau_s = F_RS;
theta.v_s = v_s;
theta.sigma_2 = sigma_2;

v_vec = 0:.01:1;
test_f = F_friction_static(v_vec,theta);
plot(v_vec,test_f,'.-')
grid on
ylim([0 80])
xlabel('velocity v')
ylabel('static friction')
ylim([0 50])

%% Plot the data points obtained via control of the simulated system

