%% set parameters and generate data
%test 

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

N_data = 50;
v_data = linspace(0.01,1,N_data);
f_data = F_friction_static(v_data,theta_true);

% add noise to data
v_data_noise = v_data+data_noise(v_data);
f_data_noise = f_data+data_noise(f_data);


%% Plot the data points obtained via control of the simulated system

% load('measurements/static_velocity_control_data_noNoise.mat')
% plot(v_points,u_points,'ok')
% legend('analytic curve','simulated points')

%% Identify static parameters via curve fitting

% goal is to find parameters theta as above.
% use matlab's fcn lsqnonlin and with the curve F_friction_static(v,theta)

% initial values for optimization varies from true value
theta_0 = theta_true*5;

% reshape data into column vec
if isrow(v_data_noise)
    v_data_noise = v_data_noise';
end
if isrow(f_data_noise)
    f_data_noise = f_data_noise';
end

% parameter bounds
theta_lb = zeros(4,1);
theta_ub = inf*ones(4,1);

% optimization
[theta_ident,resnorm] = lsqnonlin(@(theta)diff_fric(theta,v_data_noise,...
    f_data_noise),theta_0, ...
    theta_lb, theta_ub);

%% Plot the identified static friction curve and data.
N_data = 500;
v_data = linspace(0,1,N_data);

figure
plot(v_data_noise,f_data_noise,'.')
grid on
% ylim([0 80])
xlabel('velocity v')
ylabel('static friction')
% ylim([0 50])
hold on
plot(v_data,F_friction_static(v_data,theta_ident))
plot(v_data,F_friction_static(v_data,theta_true))
annotation('textbox',[.15,.6,.3,.3],'String',['\theta_{true}=(',...
    num2str(theta_true),'). \theta_{ident}=(',...
    num2str(theta_ident),').'])
legend('data','identified','true')
title('static friction curve fitting')

%% Functions

function diff = diff_fric(theta,v_data,f_data)
% returns the difference of measured (or true) friction and computed one
% via velocities v_data and parameters theta
% inputs must be:
% theta - 4x1 matrix
% v_data - Nx1 matrix
% f_data - Nx1 matrix

diff = f_data - F_friction_static(v_data,theta);

end

function data_noise = data_noise(data)
% input is a vector of a signal
% output is a uniformly distributed noise with amplitude as a percentage of
% the maximum data entry.

percent = 0.05;
data_noise = percent*max(data)*(rand(size(data))-0.5);
end