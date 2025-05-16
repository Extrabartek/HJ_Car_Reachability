function gradient_field = computeEntryTimeGradients(g, value_function_full, arrival_time, gradient_component, tau)
% COMPUTEENTRYTIMEGRADIENTS Computes gradient field based on entry time to BRS/FRS
%
% This function computes the gradient of a value function at the specific time
% each point enters the reachable set, providing a more accurate representation
% of the gradient field for trajectory planning and control determination.
%
% Inputs:
%   g                  - Grid structure from reachability analysis
%   value_function_full - Value function for all time steps (4D array for 3D space)
%   arrival_time       - Time of arrival function from compute_arrival_time.m
%                        Values are continuous times when each point entered BRS/FRS
%   gradient_component - Which gradient component to extract: 
%                        1=gamma/yaw rate, 2=beta/sideslip, 3=delta/steering
%   tau               - Time vector corresponding to value_function_full
%
% Output:
%   gradient_field    - Gradient values at entry time for specified component
%
% Example:
%   % First compute arrival time function
%   arrival_time = compute_arrival_time(data_brs_full, tau);
%   % Then compute steering gradients at entry time
%   steering_gradients = computeEntryTimeGradients(g, data_brs_full, arrival_time, 3, tau);

% Get the size of the value function (assuming same size as grid)
gradient_field = zeros(size(arrival_time));

% Mask for unreachable points
reachable_mask = isfinite(arrival_time);
gradient_field(~reachable_mask) = NaN; % Mark unreachable points as NaN

% Number of time steps
num_times = size(value_function_full, ndims(value_function_full));

% Pre-compute gradients for all time slices
fprintf('Computing gradients for all time slices...\n');
all_gradients = cell(num_times, 1);

for t = 1:num_times
    % Extract value function at this time
    if ndims(value_function_full) == 4
        % 3D state space + time
        value_t = value_function_full(:,:,:,t);
    else
        % 2D state space + time
        value_t = value_function_full(:,:,t);
    end
    
    % Compute gradients
    all_gradients{t} = computeGradients(g, value_t);
    
    % Display progress
    if mod(t, 10) == 0 || t == num_times
        fprintf('Computed gradients for time slice %d/%d\n', t, num_times);
    end
end

% Evaluate gradient at entry time for each point
fprintf('Computing entry-time gradients...\n');

% Create a time to index mapping function
time_to_idx = @(t) min(max(1, find(tau <= t, 1, 'last')), length(tau));
if isempty(time_to_idx(tau(1)))
    % Alternative mapping for edge cases
    time_to_idx = @(t) min(max(1, round(interp1(tau, 1:length(tau), t, 'nearest'))), length(tau));
end

% Total number of points to process
total_points = sum(reachable_mask(:));
points_processed = 0;
display_interval = max(1, floor(total_points / 20));  % Show progress every 5%

% For 3D state space
if ndims(value_function_full) == 4
    % Loop through all grid points
    for i = 1:size(arrival_time, 1)
        for j = 1:size(arrival_time, 2)
            for k = 1:size(arrival_time, 3)
                % Skip unreachable points
                if ~reachable_mask(i, j, k)
                    continue;
                end
                
                % Get the arrival time and convert to index
                arr_time = arrival_time(i, j, k);
                entry_idx = time_to_idx(arr_time);
                
                % Get the gradient at this entry time
                entry_gradients = all_gradients{entry_idx};
                
                % Create state vector [yaw rate, sideslip, steering angle]
                x = [g.vs{1}(i), g.vs{2}(j), g.vs{3}(k)]';
                
                % Evaluate gradient at this point and time
                grad = eval_u(g, entry_gradients, x);
                
                % Store the requested gradient component
                gradient_field(i, j, k) = grad(gradient_component);
                
                % Update progress tracking
                points_processed = points_processed + 1;
                if mod(points_processed, display_interval) == 0
                    fprintf('Processed %d/%d points (%.1f%%)\n', points_processed, total_points, ...
                        100 * points_processed / total_points);
                end
            end
        end
    end
else
    % For 2D state space
    for i = 1:size(arrival_time, 1)
        for j = 1:size(arrival_time, 2)
            % Skip unreachable points
            if ~reachable_mask(i, j)
                continue;
            end
            
            % Get the arrival time and convert to index
            arr_time = arrival_time(i, j);
            entry_idx = time_to_idx(arr_time);
            
            % Get the gradient at this entry time
            entry_gradients = all_gradients{entry_idx};
            
            % Create state vector [yaw rate, sideslip]
            x = [g.vs{1}(i), g.vs{2}(j)]';
            
            % Evaluate gradient at this point and time
            grad = eval_u(g, entry_gradients, x);
            
            % Store the requested gradient component
            gradient_field(i, j) = grad(gradient_component);
            
            % Update progress tracking
            points_processed = points_processed + 1;
            if mod(points_processed, display_interval) == 0
                fprintf('Processed %d/%d points (%.1f%%)\n', points_processed, total_points, ...
                    100 * points_processed / total_points);
            end
        end
    end
end

fprintf('Entry-time gradient calculation complete.\n');
end