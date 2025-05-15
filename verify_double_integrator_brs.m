function [rms_error, r_squared] = verify_double_integrator_brs(results_folder, varargin)
% VERIFY_DOUBLE_INTEGRATOR_BRS Verifies BRS solution against analytical parabola
%
% This function loads a computed BRS for a double integrator system,
% extracts its boundary points, and compares them with the analytical solution
% (which forms parabolas in the phase space).
%
% Inputs:
%   results_folder - Path to the BRS results folder
%   varargin - Optional parameter-value pairs:
%     'control_limit' - Control limit (default: extracted from data)
%     'target_radius' - Target set radius (default: extracted from data)
%     'time_index'    - Time index to analyze (default: final time)
%     'visualize'     - Whether to show plots (default: true)
%
% Outputs:
%   rms_error   - Root mean square error between analytical and numerical solutions
%   r_squared   - R² coefficient of determination (1 = perfect fit)
%
% Example:
%   [rms_error, r_squared] = verify_double_integrator_brs('results/doubleint_brs_results_20250507');

%% Parse inputs
p = inputParser;
p.addRequired('results_folder', @ischar);
p.addParameter('control_limit', [], @isnumeric);
p.addParameter('target_radius', [], @isnumeric);
p.addParameter('time_index', [], @isnumeric);
p.addParameter('visualize', true, @islogical);

p.parse(results_folder, varargin{:});
opts = p.Results;

%% Check if folder exists
if ~exist(results_folder, 'dir')
    error('Results folder not found: %s', results_folder);
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

% Get control limit if not specified
if isempty(opts.control_limit)
    if isfield(data, 'acceleration_limits')
        control_limit = data.acceleration_limits(1);
    elseif isfield(data, 'sim_params') && isfield(data.sim_params, 'acceleration_limits')
        control_limit = data.sim_params.acceleration_limits(1);
    else
        error('Control limit not found in data. Please specify using ''control_limit'' parameter.');
    end
else
    control_limit = opts.control_limit;
end

% Get target radius if not specified
if isempty(opts.target_radius)
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
else
    target_radius = opts.target_radius;
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

% If data is 3D (includes time), get the specified time slice
if ndims(value_function) == 3
    if isempty(opts.time_index)
        % Default to final time
        time_idx = size(value_function, 3);
    else
        time_idx = opts.time_index;
    end
    
    if time_idx > size(value_function, 3)
        error('Time index exceeds available time steps.');
    end
    
    value_function_slice = value_function(:,:,time_idx);
    
    % Get corresponding time if available
    if isfield(data, 'tau')
        time_value = data.tau(time_idx);
        fprintf('Analyzing time slice %d of %d (t = %.2f s)\n', ...
            time_idx, size(value_function, 3), time_value);
    else
        fprintf('Analyzing time slice %d of %d\n', time_idx, size(value_function, 3));
    end
else
    % Single time slice
    value_function_slice = value_function;
    fprintf('Analyzing single time slice (final BRS)\n');
end

fprintf('Using control limit: %.2f, target radius: %.2f\n', control_limit, target_radius);

%% Extract BRS boundary points
% The zero level set of the value function represents the boundary
% Use the marching squares algorithm to find the boundary

% First create a binary mask of the BRS (all points with value <= 0)
brs_mask = (value_function_slice <= 0);

% Replace the contourc line (around line 149) with this block
% First, examine the structure of the grid
if isstruct(g) && isfield(g, 'xs')
    % Check dimensions and types
    if iscell(g.xs) && length(g.xs) >= 2
        % Try to extract grid information safely
        if isvector(g.xs{1}) && isvector(g.xs{2})
            % If g.xs contains vectors, use them directly
            [X, Y] = meshgrid(g.xs{1}, g.xs{2});
            C = contourc(g.xs{1}, g.xs{2}, value_function_slice', [0, 0]);
        else
            % If g.xs contains matrices or higher-dimensional arrays
            % Generate simple coordinate vectors based on array size
            [rows, cols] = size(value_function_slice);
            x_vec = 1:cols;
            y_vec = 1:rows;
            % Create a simple contour without actual coordinates
            C = contourc(x_vec, y_vec, value_function_slice, [0, 0]);
            % Note that this will give relative positions, not actual coordinates
        end
    else
        % Fallback to direct contour
        [c, ~] = contour(value_function_slice, [0, 0]);
        C = c;
    end
else
    % Fallback to direct contour if g structure is unexpected
    [c, ~] = contour(value_function_slice, [0, 0]);
    C = c;
end

% Process contour data
boundary_points = [];
idx = 1;
while idx < size(C, 2)
    level = C(1, idx);
    count = C(2, idx);
    if level == 0  % This is a zero level contour
        next_points = C(:, idx+1:idx+count)';
        boundary_points = [boundary_points; next_points];
    end
    idx = idx + count + 1;
end

if isempty(boundary_points)
    error('Could not extract BRS boundary points. Check if BRS exists.');
end

fprintf('Extracted %d boundary points\n', size(boundary_points, 1));

% After the boundary points extraction (around line 160-170), replace or add this code:

% Convert boundary points from grid indices to actual coordinates
transformed_boundary_points = zeros(size(boundary_points));

% Determine grid center indices
[rows, cols] = size(value_function_slice);
center_row = ceil(rows/2);
center_col = ceil(cols/2);

fprintf('Grid dimensions: %d x %d, Center: (%d, %d)\n', rows, cols, center_row, center_col);

for i = 1:size(boundary_points, 1)
    % Extract the grid indices (might need rounding)
    col_idx = round(boundary_points(i, 1));
    row_idx = round(boundary_points(i, 2));
    
    % Ensure indices are within bounds
    col_idx = max(1, min(col_idx, cols));
    row_idx = max(1, min(row_idx, rows));
    
    % Get the actual coordinate values from the grid
    x_val = g.xs{1}(row_idx, col_idx);  % Position
    v_val = g.xs{2}(row_idx, col_idx);  % Velocity
    
    transformed_boundary_points(i, 1) = x_val;
    transformed_boundary_points(i, 2) = v_val;
end

% Replace original boundary points with transformed ones
boundary_points = transformed_boundary_points;

fprintf('Transformed boundary points to actual coordinates\n');
fprintf('Coordinate ranges: x=[%.4f, %.4f], v=[%.4f, %.4f]\n', ...
    min(boundary_points(:,1)), max(boundary_points(:,1)), ...
    min(boundary_points(:,2)), max(boundary_points(:,2)));

% Display a few transformed points
fprintf('First few transformed boundary points:\n');
disp(boundary_points(1:min(5, size(boundary_points, 1)), :));

% Check if we still need to swap columns based on expected ranges
var_x = var(boundary_points(:, 1));
var_v = var(boundary_points(:, 2));
fprintf('Variance: x=%.4f, v=%.4f\n', var_x, var_v);

% Now continue with the rest of the code...

% Now continue with the separation of points into positive and negative velocity
pos_vel_mask = boundary_points(:, 2) >= 0;
neg_vel_mask = boundary_points(:, 2) < 0;

pos_vel_points = boundary_points(pos_vel_mask, :);
neg_vel_points = boundary_points(neg_vel_mask, :);

fprintf('Points with v ≥ 0: %d, Points with v < 0: %d\n', ...
    sum(pos_vel_mask), sum(neg_vel_mask));

%% Fit analytical parabolas
% For v ≥ 0: x = -v²/(2*umax) - offset  (braking parabola)
% For v < 0: x = v²/(2*umax) + offset   (accelerating parabola)

% Fit for positive velocity region
if ~isempty(pos_vel_points)
    v_pos = pos_vel_points(:, 2);
    x_pos = pos_vel_points(:, 1);
    
    % Define the analytical model: x = -v²/(2*umax) - offset
    model_pos = @(offset, v) -v.^2 / (2 * control_limit) - offset;
    
    % Use optimization to find best offset
    cost_func_pos = @(offset) sum((model_pos(offset, v_pos) - x_pos).^2);
    offset_pos = fminsearch(cost_func_pos, target_radius);
    
    % Calculate predictions and errors
    x_pred_pos = model_pos(offset_pos, v_pos);
    error_pos = x_pred_pos - x_pos;
    sse_pos = sum(error_pos.^2);
    sst_pos = sum((x_pos - mean(x_pos)).^2);
    r2_pos = 1 - sse_pos/sst_pos;
    rmse_pos = sqrt(mean(error_pos.^2));
    
    fprintf('Positive velocity fit: offset = %.4f (expected: ~%.4f)\n', offset_pos, target_radius);
    fprintf('  RMSE = %.4f, R² = %.4f\n', rmse_pos, r2_pos);
else
    fprintf('No positive velocity boundary points found.\n');
    offset_pos = NaN;
    rmse_pos = NaN;
    r2_pos = NaN;
end

% Fit for negative velocity region
if ~isempty(neg_vel_points)
    v_neg = neg_vel_points(:, 2);
    x_neg = neg_vel_points(:, 1);
    
    % Define the analytical model: x = v²/(2*umax) + offset
    model_neg = @(offset, v) v.^2 / (2 * control_limit) + offset;
    
    % Use optimization to find best offset
    cost_func_neg = @(offset) sum((model_neg(offset, v_neg) - x_neg).^2);
    offset_neg = fminsearch(cost_func_neg, target_radius);
    
    % Calculate predictions and errors
    x_pred_neg = model_neg(offset_neg, v_neg);
    error_neg = x_pred_neg - x_neg;
    sse_neg = sum(error_neg.^2);
    sst_neg = sum((x_neg - mean(x_neg)).^2);
    r2_neg = 1 - sse_neg/sst_neg;
    rmse_neg = sqrt(mean(error_neg.^2));
    
    fprintf('Negative velocity fit: offset = %.4f (expected: ~%.4f)\n', offset_neg, target_radius);
    fprintf('  RMSE = %.4f, R² = %.4f\n', rmse_neg, r2_neg);
else
    fprintf('No negative velocity boundary points found.\n');
    offset_neg = NaN;
    rmse_neg = NaN;
    r2_neg = NaN;
end

% Calculate overall metrics (weighted by number of points)
n_pos = sum(pos_vel_mask);
n_neg = sum(neg_vel_mask);
n_total = n_pos + n_neg;

if n_total > 0
    rms_error = sqrt((n_pos * rmse_pos^2 + n_neg * rmse_neg^2) / n_total);
    r_squared = (n_pos * r2_pos + n_neg * r2_neg) / n_total;
    
    fprintf('\nOverall fit quality: RMSE = %.4f, R² = %.4f\n', rms_error, r_squared);
else
    rms_error = NaN;
    r_squared = NaN;
    fprintf('\nCould not calculate overall metrics (no boundary points).\n');
end

%% Visualize if requested
if opts.visualize
    figure('Name', 'Double Integrator BRS Verification', 'Position', [100, 100, 1000, 700]);
    
    % Plot the BRS using filled contour
    contourf(g.xs{1}, g.xs{2}, value_function_slice', [0, 0], 'LineStyle', 'none');
    colormap(flipud(gray));
    hold on;
    
    % Plot the BRS boundary (numerical solution)
    contour(g.xs{1}, g.xs{2}, value_function_slice', [0, 0], 'LineWidth', 2, 'Color', 'b');
    
    % Create a denser grid for plotting analytical parabolas
    % Use the actual coordinate ranges instead of g.xs directly
    v_dense = linspace(min(boundary_points(:,2))-0.5, max(boundary_points(:,2))+0.5, 1000);
    
    % Also update the target set visualization (around line 365)
    % Instead of using rectangle which may not be positioned correctly
    target_center = [0, 0]; % Assuming target is centered at origin
    if isstruct(g) && isfield(g, 'center')
        target_center = g.center;
    end
    rectangle('Position', [target_center(1)-target_radius, target_center(2)-target_radius, ...
                      2*target_radius, 2*target_radius], ...
         'Curvature', [1, 1], 'LineWidth', 2, 'LineStyle', '--', 'EdgeColor', 'g');
    
    % Plot analytical solutions
    % Positive velocity parabola
    v_pos_dense = v_dense(v_dense >= 0);
    if ~isempty(v_pos_dense) && ~isnan(offset_pos)
        x_pos_dense = -v_pos_dense.^2 / (2 * control_limit) - offset_pos;
        plot(x_pos_dense, v_pos_dense, 'r-', 'LineWidth', 2);
    end
    
    % Negative velocity parabola
    v_neg_dense = v_dense(v_dense < 0);
    if ~isempty(v_neg_dense) && ~isnan(offset_neg)
        x_neg_dense = v_neg_dense.^2 / (2 * control_limit) + offset_neg;
        plot(x_neg_dense, v_neg_dense, 'r-', 'LineWidth', 2);
    end
    
    % Plot the target set
    rectangle('Position', [-target_radius, -target_radius, 2*target_radius, 2*target_radius], ...
             'Curvature', [1, 1], 'LineWidth', 2, 'LineStyle', '--', 'EdgeColor', 'g');
    
    % Add labels and legend
    xlabel('Position', 'FontSize', 14);
    ylabel('Velocity', 'FontSize', 14);
    title(sprintf('Double Integrator BRS Verification (u_{max} = %.2f)', control_limit), 'FontSize', 16);
    legend('BRS Region', 'Numerical Boundary', 'Analytical Boundary', 'Target Set', 'Location', 'best');
    
    % Add fit quality text
    text_str = sprintf('Fit Quality:\nRMSE = %.4f\nR² = %.4f', rms_error, r_squared);
    annotation('textbox', [0.7, 0.8, 0.2, 0.1], 'String', text_str, ...
               'EdgeColor', 'none', 'BackgroundColor', 'w', 'FontSize', 12);
    
    grid on;
    axis equal;
    
    % Also create a residual plot
    figure('Name', 'Residual Analysis', 'Position', [200, 200, 900, 400]);
    
    % Positive velocity residuals
    subplot(1, 2, 1);
    if ~isempty(pos_vel_points) && ~isnan(offset_pos)
        scatter(v_pos, error_pos, 30, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
        hold on;
        plot(xlim(), [0, 0], 'k--', 'LineWidth', 1.5);
        xlabel('Velocity', 'FontSize', 12);
        ylabel('Residual (Analytical - Numerical)', 'FontSize', 12);
        title('Residuals for v ≥ 0', 'FontSize', 14);
        grid on;
    else
        text(0.5, 0.5, 'No data available', 'HorizontalAlignment', 'center');
        axis off;
    end
    
    % Negative velocity residuals
    subplot(1, 2, 2);
    if ~isempty(neg_vel_points) && ~isnan(offset_neg)
        scatter(v_neg, error_neg, 30, 'r', 'filled', 'MarkerFaceAlpha', 0.6);
        hold on;
        plot(xlim(), [0, 0], 'k--', 'LineWidth', 1.5);
        xlabel('Velocity', 'FontSize', 12);
        ylabel('Residual (Analytical - Numerical)', 'FontSize', 12);
        title('Residuals for v < 0', 'FontSize', 14);
        grid on;
    else
        text(0.5, 0.5, 'No data available', 'HorizontalAlignment', 'center');
        axis off;
    end
end

end