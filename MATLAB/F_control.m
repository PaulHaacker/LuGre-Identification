function F_an = F_control(t,x,parameter)
% antreibende kraft, control input
% switch variable for control type

% switchVar = 'FF_ramp';
% switchVar = 'FB_PI_velocity_control';
% switchVar = 'FB_PI_velocity_control_FF_breakaway';
% switchVar = 'FB_PI_velocity_control_FF_breakaway_Schedule_setpoint';
% switchVar = 'FF_swelling';

switchVar = parameter.controller.mode;

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
    case 'FF_swelling'
%         % experimentally identified break-away force for FF
%         F_BreakAway = 20;
%         
%         F_FF = 100*t;
%         if F_FF > F_BreakAway
%             F_FF = F_BreakAway;
%         end
%         
%         F_an = F_FF + (10-10*cos(t));
        % experimentally identified break-away force for FF

%         F_an = 20*(1-cos(t));
        F_an = 40*(sin(t));
    otherwise
        F_an = 0;
        warning('Could not read control input type in fcn F_an')
end


end