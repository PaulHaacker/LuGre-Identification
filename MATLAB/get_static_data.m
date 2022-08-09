function [u_points,v_points] = get_static_data(t,v,u)
%get_static_data: returns the measured static data points
%(u_points,v_points) from the input arguments time t, velocity trajectory 
% v, control input u 
%   Detailed explanation goes here
    acc = diff(v)./diff(t); % of length equal length(v)-1
    [b,a]=butter(1,.2);
    acc_filter = filtfilt(b,a,acc);
    
    indx = abs(acc_filter)<1e-3; % logical array of size equal size(acc_filter)
    indx_boundaries = diff(indx); % array w/ entries -1, 1 or 0 of length(acc_filter)-1
    % indicating the start (1) or stop (-1) of a constant velocity interval
    indx_nr_startConstInterval = find(indx_boundaries==1); % index number of
    % a start event
    indx_nr_endConstInterval = find(indx_boundaries==-1); % index number of 
    % a stop event
    if length(indx_nr_startConstInterval)>length(indx_nr_endConstInterval)
        % catch the case where the measurement ends within an interval.
        indx_nr_endConstInterval = [indx_nr_endConstInterval;...
            length(indx_boundaries)];
    end
    time_instance_start = t(indx_nr_startConstInterval); % time instance at
    % the start event
    time_instance_end = t(indx_nr_endConstInterval); % time instance at the
    % stop event
    if length(time_instance_start)~=length(time_instance_end)
        warning('Error when detecting starts and stops of intervals. Quitting.')
        u_points = [];
        v_points = [];
    else
        % delete intervals that are too short.
        interval_lengths = time_instance_end - time_instance_start;
        interval_length_threshold = .1;
        indx_nr_startConstInterval(interval_lengths<interval_length_threshold) = [];
        indx_nr_endConstInterval(interval_lengths<interval_length_threshold) = [];
        
        u_points = zeros(size(indx_nr_startConstInterval));
        v_points = zeros(size(indx_nr_startConstInterval));
        
        for k = 1:length(indx_nr_startConstInterval)
            u_points(k) = mean(u(indx_nr_startConstInterval(k):...
                indx_nr_endConstInterval(k)));
            v_points(k) = mean(v(indx_nr_startConstInterval(k):...
                indx_nr_endConstInterval(k)));
        end
        
        % plots for sanity check.
        figure
        plot(t,v)
        hold on
        for k = 1:length(indx_nr_startConstInterval)
            time_help = t(indx_nr_startConstInterval(k):...
                indx_nr_endConstInterval(k));
            plot(time_help, v_points(k)*ones(size(time_help)),'r')
        end
        ylabel('velocity')
        xlabel('time')
        grid on
    end
end

