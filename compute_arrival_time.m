function arrival_time = compute_arrival_time(data_brs, tau)
% COMPUTE_ARRIVAL_TIME Computes the time-of-arrival function from BRS data
%
% Inputs:
%   data_brs - Value function data from BRS computation (4D array)
%   tau     - Time vector corresponding to data_brs
%
% Outputs:
%   arrival_time - Time-of-arrival function (3D array, same size as data_brs(:,:,:,1))
%                  Values are times when each point first entered the BRS
%                  Points outside the BRS have value inf

% Get dimensions from the value function
dims = size(data_brs);
if ndims(data_brs) == 4
    % 3D state space with time as 4th dimension
    grid_dims = dims(1:3);
    num_times = dims(4);
else
    % 2D state space with time as 3rd dimension
    grid_dims = dims(1:2);
    num_times = dims(3);
end

% Initialize arrival time with infinity (points outside BRS)
arrival_time = inf(grid_dims);

% Process each time step in reverse (from final time to initial time)
for t_idx = num_times:-1:1
    % Extract value function at current time
    if ndims(data_brs) == 4
        current_data = data_brs(:,:,:,t_idx);
    else
        current_data = data_brs(:,:,t_idx);
    end
    
    % Find points in the BRS at this time (value function <= 0)
    in_brs = (current_data <= 0);
    
    % For points not yet assigned an arrival time, record this time
    unassigned = isfinite(arrival_time) == false;
    new_points = in_brs & unassigned;
    
    % Assign arrival time for these new points
    arrival_time(new_points) = tau(t_idx);
    
    % Optional: Display progress
    if mod(t_idx, 10) == 0
        fprintf('Computing arrival time: %.1f%% complete\n', 100*(num_times-t_idx+1)/num_times);
    end
end

% Points still unassigned (arrival_time == inf) remain outside the BRS
fprintf('Time-of-arrival function computed. %.2f%% of points are reachable.\n', ...
    100*sum(isfinite(arrival_time(:)))/numel(arrival_time));

end