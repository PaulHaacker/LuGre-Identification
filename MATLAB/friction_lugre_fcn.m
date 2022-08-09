function [t,x] = friction_lugre_fcn(tspan,x_0)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
% test lugre friction - simple 1D body with friction
close all

% x_0 = [0 0 0]';
opts = odeset('MaxStep',1e-3);
[t,x] = ode23tb(@ode,tspan,x_0,opts);

% plot(t,x(:,2),'.-')
% legend('velocity')
% xlabel('time t')
% grid on
% movegui('northwest')
% figure
% plot(t,x(:,3),'.-')
% legend('LuGre displacement')
% grid on
% movegui('northeast')
% 
% % plot der reibkraft und control input
% u_control = zeros(size(t));
% for k = 1:length(t)
%     u_control(k) = F_an(t(k),x(k,:));
% end
% 
% v = x(:,2);
% z = x(:,3);
% z_dot = v-abs(v)./g_fric(v).*z;
% F_R_num = F_R(v,z,z_dot);
% figure
% plot(v,F_R_num,'.-')
% hold on
% plot(v,u_control,'.-')
% xlabel('velocity v')
% legend('friction F_R','control input u')
% grid on
% movegui('southeast')
% 
% % plot of step size
% figure
% plot(t(2:end)-t(1:end-1))
% ylabel('step size')
% xlabel('step number')
% movegui('southwest')

end

function dx = ode(t,x)

% first order ODE, state is body position y & velocity y_dot and bristle
% deplacement z. x = [ y ; y_dot ; z ]
dx = zeros(size(x));

m = 5;

v = x(2);
z = x(3);

dx(1) = v;
dx(3) = v-abs(v)/g_fric(v)*z;

z_dot = dx(3);
dx(2) = (F_an(t,x)-F_R(v,z,z_dot))/m;

end

function F_an = F_an(t,x)
% antreibende kraft, control input
% switch variable for control type

% switchVar = 'FF_ramp';
% switchVar = 'FB_PI_velocity_control';
% switchVar = 'FB_PI_velocity_control_FF_breakaway';
switchVar = 'FB_PI_velocity_control_FF_breakaway_Schedule_setpoint';

switch switchVar
    case 'FF_ramp'
        % FF ramp
        if t<inf
            F_an = 100*t;
        else
            F_an = 0;
        end
    case 'FB_PI_velocity_control'
        
        % FB velocity PI control (using q_dot and q)
        gain_P = 1000;
        gain_I = 100;
        
        % following only works for constant desired velocities! otherwise
        % need to introduce a new state for the I-part of control. (easier 
        % in simulink)
        
        des_vel = .01;
        des_pos = des_vel*t;
        
        act_vel = x(2);
        act_pos = x(1);
        
        error_vel = des_vel -act_vel;
        error_pos = des_pos -act_pos;
        
        F_an = gain_P*error_vel + gain_I*error_pos;
        
    case 'FB_PI_velocity_control_FF_breakaway'
        
        % FB velocity PI control (using q_dot and q)
        gain_P = 1000;
        gain_I = 100*0;
        
        % experimentally identified break-away force for FF
        F_BreakAway = 20;
        
        % following only works for constant desired velocities! otherwise
        % need to introduce a new state for the I-part of control. (easier 
        % in simulink)
        
        des_vel = 1;
        des_pos = des_vel*t;
        
        act_vel = x(2);
        act_pos = x(1);
        
        error_vel = des_vel -act_vel;
        error_pos = des_pos -act_pos;
        
        F_an = gain_P*error_vel + gain_I*error_pos;
        
        F_FF = 100*t;
        if F_FF > F_BreakAway
            F_FF = F_BreakAway;
        end
        F_an = F_an + F_FF;
    case 'FB_PI_velocity_control_FF_breakaway_Schedule_setpoint'
        
        % FB velocity PI control (using q_dot and q)
        gain_P = 500;
        gain_I = 100*0;
        
        % experimentally identified break-away force for FF
        F_BreakAway = 20;
        
        % setpoint scheduling
        number_setpoints = 20;
        time_step = 0.5;
        v_min = .01;
        v_max = 1;
        des_vel_vec = linspace(v_min,v_max,number_setpoints);
        time_schedule = 0:time_step:time_step*number_setpoints;
        
        if isempty(find(time_schedule>t,1))
            des_vel = des_vel_vec(end);
        else
            indx_time = find(time_schedule>t,1,'first');
            des_vel = des_vel_vec(indx_time-1);
        end
        
        % following only works for constant desired velocities! otherwise
        % need to introduce a new state for the I-part of control. (easier 
        % in simulink)
        
        des_pos = des_vel*t;
        
        act_vel = x(2);
        act_pos = x(1);
        
        error_vel = des_vel -act_vel;
        error_pos = des_pos -act_pos;
        
        F_an = gain_P*error_vel + gain_I*error_pos;
        
        F_FF = 100*t;
        if F_FF > F_BreakAway
            F_FF = F_BreakAway;
        end
        F_an = F_an + F_FF;
    otherwise
        F_an = 0;
        warning('Could not read control input type in fcn F_an')
end


end

function F_R = F_R(v,z,z_dot)
% reibkraft

[parameter,~] = paramter_fcn();

sigma_0 = parameter(1);
sigma_1 = parameter(2);
sigma_2 = parameter(3);

F_R = sigma_0*z + sigma_1*z_dot + sigma_2*v;
end

function g_fric = g_fric(v)

[~,parameter] = paramter_fcn();

F_RC = parameter(1);
F_RS = parameter(2);
v_s = parameter(3);
sigma_0 = parameter(4);
g_fric = (F_RC + (F_RS-F_RC)*exp(-(v/v_s).^2))/sigma_0;
end

function [parameters_1,parameters_2] = paramter_fcn()
% F_RC = 1;
% F_RS = 1.5;
% v_s = 1e-3;
% sigma_0 = 1e5;
% 
% sigma_1 = sqrt(sigma_0);
% sigma_2 = .4;

F_RC = 5;
F_RS = 20;
v_s = .07;
sigma_0 = 133.3e3;

sigma_1 = 600;
sigma_2 = 35;

parameters_2 = [F_RC,F_RS,v_s,sigma_0];
parameters_1 = [sigma_0,sigma_1,sigma_2];

end

