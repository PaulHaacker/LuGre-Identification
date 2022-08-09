%% data processing for curve fitting

load('measurements/measurement_velocity_control_time_noNoise.mat')
% measurement_velocity_control_time_noNoise.mat contains the following
% measured signals: time t, system input u_control, velocity v. all of same
% dimension.

[u_points,v_points] = get_static_data(t,v,u_control);