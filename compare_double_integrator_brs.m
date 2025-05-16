function compare_double_integrator_brs(results_folder)
% COMPARE_DOUBLE_INTEGRATOR_BRS Compares a numerically computed double integrator BRS
% against the analytical solution (parabolas)
%
% Input:
%   results_folder - Path to folder containing BRS computation results
%
% Example:
%   compare_double_integrator_brs('results/doubleint_brs_results_20250507_142451')

%% Validate input
if nargin < 1 || ~exist(results_folder, 'dir')
    error('Please provide a valid results folder path');
end

%% Load the BRS data
fprintf('Loading BRS data from %s...\n', results_folder);

% First try loading combined results
combined_file = fullfile(results_folder, 'brs_combined_results.mat');
if ~exist(combined_file, 'file')
    % Try to find any brs_*.mat file
    files = dir(fullfile(results_folder, 'brs_*.mat'));
    if isempty(files)
        error('No BRS data files found in the specified folder');
    end
    data_file = fullfile(results_folder, files(1).name);
    fprintf('Loading individual BRS file: %s\n', data_file);
    data = load(data_file);
else
    data = load(combined_file);
end

%% Extract necessary data
% Get grid information
g = data.g;

% Get control limit
if isfield(data, 'acceleration_limits')
    control_limit = data.acceleration_limits(1);
elseif isfield(data, 'sim_params') && isfield(data.sim_params, 'acceleration_limits')
    control_limit = data.sim_params.acceleration_limits(1);
else
    % Prompt user for control limit if not found
    control_limit = input('Control limit (umax) not found. Please enter the value: ');
end

% Get target radius
if isfield(data, 'targetSize')
    target_radius = data.targetSize(1);
elseif isfield(data, 'sim_params') && isfield(data.sim_params, 'target_size')
    target_radius = data.sim_params.target_size(1);
else
    % Try to estimate from data0
    if isfield(data, 'data0')
        % Find the maximum position with value <= 0 when velocity = 0
        vel_zero_idx = ceil(size(data.data0, 2)/2);  % Middle index for velocity
        position_values = data.data0(:, vel_zero_idx);
        zero_positions = find(position_values <= 0);
        if ~isempty(zero_positions)
            target_radius = max(abs(g.xs{1}(zero_positions)));
        else
            target_radius = 0.5;  % Default
            warning('Could not estimate target radius. Using default: 0.5');
        end
    else
        target_radius = 0.5;  % Default
        warning('Target radius not found in data. Using default: 0.5');
    end
end

% Get value function data
if isfield(data, 'all_data')
    % For combined results, use the first velocity and control limit
    value_function = data.all_data{1, 1};
elseif isfield(data, 'data')
    % For individual results, use the data field
    value_function = data.data;
else
    error('Value function data not found.');
end

% If data is 3D (includes time), use the final time slice
if ndims(value_function) == 3
    % Use final time
    value_function_slice = value_function(:,:,end);
    
    % Get corresponding time if available
    if isfield(data, 'tau')
        time_value = data.tau(end);
        fprintf('Using final time slice (t = %.2f s)\n', time_value);
    else
        fprintf('Using final time slice\n');
    end
else
    % Single time slice
    value_function_slice = value_function;
    fprintf('Using single time slice (final BRS)\n');
end

fprintf('Using control limit: %.2f, target radius: %.2f\n', control_limit, target_radius);

%% Create figure for visualization
figure('Name', 'Double Integrator BRS Comparison', 'Position', [100, 100, 1000, 700]);

% Plot the BRS using filled contour
contourf(g.xs{1}, g.xs{2}, value_function_slice', [0, 0], 'LineStyle', 'none');
colormap(flipud(gray)); % Dark area represents the BRS
hold on;

% Plot the BRS boundary (numerical solution)
contour(g.xs{1}, g.xs{2}, value_function_slice', [0, 0], 'LineWidth', 2, 'Color', 'b');

% Create a dense grid for plotting analytical parabolas
% Use the grid ranges from the data
x_min = min(g.xs{1}(:));
x_max = max(g.xs{1}(:));
v_min = min(g.xs{2}(:));
v_max = max(g.xs{2}(:));

v_dense = linspace(v_min, v_max, 1000);

% Compute analytical boundaries
% For positive velocities: x = -v²/(2*umax) - r
v_pos_dense = v_dense(v_dense >= 0);
if ~isempty(v_pos_dense)
    x_pos_dense = -v_pos_dense.^2 / (2 * control_limit) - target_radius;
    plot(x_pos_dense, v_pos_dense, 'r-', 'LineWidth', 2);
end

% For negative velocities: x = v²/(2*umax) + r
v_neg_dense = v_dense(v_dense < 0);
if ~isempty(v_neg_dense)
    x_neg_dense = v_neg_dense.^2 / (2 * control_limit) + target_radius;
    plot(x_neg_dense, v_neg_dense, 'r-', 'LineWidth', 2);
end

% Plot the target set
rectangle('Position', [-target_radius, -target_radius, 2*target_radius, 2*target_radius], ...
         'Curvature', [1, 1], 'LineWidth', 2, 'LineStyle', '--', 'EdgeColor', 'g');

% Add labels and legend
xlabel('Position', 'FontSize', 14);
ylabel('Velocity', 'FontSize', 14);
title(sprintf('Double Integrator BRS Comparison (u_{max} = %.2f, r = %.2f)', ...
              control_limit, target_radius), 'FontSize', 16);
legend('BRS Region', 'Numerical Boundary', 'Analytical Boundary', 'Target Set', 'Location', 'best');
grid on;
axis equal;

%% Optionally compute and display error metrics
% Calculate analytical error metrics (optional)
try
    % Extract boundary points from the numerical solution
    [c, ~] = contour(g.xs{1}, g.xs{2}, value_function_slice', [0, 0]);
    
    % Process contour data
    boundary_points = [];
    idx = 1;
    while idx < size(c, 2)
        level = c(1, idx);
        count = c(2, idx);
        if level == 0  % This is a zero level contour
            next_points = c(:, idx+1:idx+count)';
            boundary_points = [boundary_points; next_points];
        end
        idx = idx + count + 1;
    end
    
    if ~isempty(boundary_points)
        % Separate points into positive and negative velocity regions
        pos_vel_mask = boundary_points(:, 2) >= 0;
        neg_vel_mask = boundary_points(:, 2) < 0;
        
        pos_vel_points = boundary_points(pos_vel_mask, :);
        neg_vel_points = boundary_points(neg_vel_mask, :);
        
        % Calculate errors
        errors = [];
        
        % Positive velocity errors
        if ~isempty(pos_vel_points)
            v_pos = pos_vel_points(:, 2);
            x_pos = pos_vel_points(:, 1);
            x_analytical_pos = -v_pos.^2 / (2 * control_limit) - target_radius;
            errors_pos = x_analytical_pos - x_pos;
            errors = [errors; errors_pos];
        end
        
        % Negative velocity errors
        if ~isempty(neg_vel_points)
            v_neg = neg_vel_points(:, 2);
            x_neg = neg_vel_points(:, 1);
            x_analytical_neg = v_neg.^2 / (2 * control_limit) + target_radius;
            errors_neg = x_analytical_neg - x_neg;
            errors = [errors; errors_neg];
        end
        
        % Compute RMS error
        rms_error = sqrt(mean(errors.^2));
        
        % Display error information
        error_info = sprintf('RMS Error: %.4f', rms_error);
        annotation('textbox', [0.7, 0.8, 0.2, 0.1], 'String', error_info, ...
                   'EdgeColor', 'none', 'BackgroundColor', 'w', 'FontSize', 12);
    end
catch err
    fprintf('Could not compute error metrics: %s\n', err.message);
end

fprintf('Visualization complete.\n');
end