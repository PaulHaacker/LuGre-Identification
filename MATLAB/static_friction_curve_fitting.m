%% Plot the static friction curve with known, hard coded parameters.

F_RC = 5;
F_RS = 20;
v_s = .07;
sigma_2 = 35;

% theta.tau_c = F_RC;
% theta.tau_s = F_RS;
% theta.v_s = v_s;
% theta.sigma_2 = sigma_2;

% lsqnonlin can only handle matrices as parameters
theta_true(1) = F_RC;
theta_true(2) = F_RS;
theta_true(3) = v_s;
theta_true(4) = sigma_2;

v_data = 0:.01:1;
f_data = F_friction_static(v_data,theta_true);
figure
plot(v_data,f_data,'.-')
grid on
ylim([0 80])
xlabel('velocity v')
ylabel('static friction')
ylim([0 50])
hold on

%% Plot the data points obtained via control of the simulated system

load('measurements/static_velocity_control_data_noNoise.mat')
plot(v_points,u_points,'ok')
legend('analytic curve','simulated points')

%% Identify static parameters via curve fitting

% goal is to find parameters theta as above.
% use matlab's fcn lsqnonlin and with the curve F_friction_static(v,theta)

% initial values for optimization varies from true value
theta_0 = theta_true*1.5;

theta_ident = lsqnonlin(

function diff = diff_fric(theta,v_data,f_data)
% returns the difference of measured (or true) friction and computed one
% via velocities v_data and parameters theta
% inputs must be:
% theta - 4x1 matrix
% v_data - Nx1 matrix
% f_data - Nx1 matrix

diff = f_data - F_friction_static(v_data,theta);

end