function data_noise = data_noise(data)
% input is a vector of a signal
% output is a uniformly distributed noise with amplitude as a percentage of
% the maximum data entry.

percent = 0.05;
data_noise = percent*max(data)*(rand(size(data))-0.5);
end