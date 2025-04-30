function result_folder = compute_safe_set(brs_folder, frs_folder, varargin)
% COMPUTE_SAFE_SET Computes the safe set as the intersection of BRS and FRS
%
% This function takes BRS and FRS results and computes their intersection to
% identify the safe set (states that are both reachable from some initial states
% and can reach the target).
%
% Inputs:
%   brs_folder      - Folder containing BRS computation results
%   frs_folder      - Folder containing FRS computation results
%   varargin        - Optional parameter-value pairs:
%                      'visualize' - Whether to visualize results (default: true)
%                      'save_results' - Whether to save results (default: true)
%                      'velocity_idx' - Velocity index to use (default: 1)
%                      'control_idx' - Control limit index to use (default: 1)
%
% Output:
%   result_folder   - Path to the folder where results are saved
%
% Example:
%   result_folder = compute_safe_set('path/to/brs_folder', 'path/to/frs_folder');

%% Parse inputs
p = inputParser;
p.addRequired('brs_folder', @ischar);
p.addRequired('frs_folder', @ischar);
p.addParameter('visualize', true, @islogical);
p.addParameter('save_results', false, @islogical);
p.addParameter('velocity_idx', 1, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.addParameter('control_idx', 1, @(x) isnumeric(x) && isscalar(x) && x > 0);

p.parse(brs_folder, frs_folder, varargin{:});
opts = p.Results;

%% Validation of input folders
if ~exist(brs_folder, 'dir')
    error('BRS folder does not exist: %s', brs_folder);
end

if ~exist(frs_folder, 'dir')
    error('FRS folder does not exist: %s', frs_folder);
end

%% Load BRS and FRS data
fprintf('Loading BRS data from: %s\n', brs_folder);
brs_combined_file = fullfile(brs_folder, 'brs_combined_results.mat');
if ~exist(brs_combined_file, 'file')
    error('BRS combined results file not found: %s', brs_combined_file);
end
brs_data = load(brs_combined_file);

fprintf('Loading FRS data from: %s\n', frs_folder);
frs_combined_file = fullfile(frs_folder, 'frs_combined_results.mat');
if ~exist(frs_combined_file, 'file')
    % Try alternate name
    frs_combined_file = fullfile(frs_folder, 'brs_combined_results.mat');
    if ~exist(frs_combined_file, 'file')
        error('FRS combined results file not found in the folder');
    end
end
frs_data = load(frs_combined_file);

%% Extract key information and verify compatibility
% Get grid and determine model type (2D or 3D)
g = brs_data.g;
is_3d_model = (length(g.N) == 3);

% Check grid compatibility
if ~isequal(g.N, frs_data.g.N) || ~isequal(g.min, frs_data.g.min) || ~isequal(g.max, frs_data.g.max)
    error('BRS and FRS grids are not compatible');
end

% Get control type
if is_3d_model
    % 3D model (steered bicycle)
    if isfield(brs_data, 'dvmax_values')
        brs_control_limits = brs_data.dvmax_values;
        control_type = 'dv';
    else
        error('Could not identify control type for 3D model');
    end
    
    if isfield(frs_data, 'dvmax_values')
        frs_control_limits = frs_data.dvmax_values;
    else
        error('Could not identify control type for FRS with 3D model');
    end
else
    % 2D model (standard bicycle)
    if isfield(brs_data, 'mzmax_values')
        brs_control_limits = brs_data.mzmax_values;
        control_type = 'mz';
    else
        error('Could not identify control type for 2D model');
    end
    
    if isfield(frs_data, 'mzmax_values')
        frs_control_limits = frs_data.mzmax_values;
    else
        error('Could not identify control type for FRS with 2D model');
    end
end

% Get velocities and check compatibility
brs_velocities = brs_data.velocities;
frs_velocities = frs_data.velocities;

% Validate indices
if opts.velocity_idx > length(brs_velocities) || opts.velocity_idx > length(frs_velocities)
    error('Velocity index exceeds available velocities');
end

if opts.control_idx > length(brs_control_limits) || opts.control_idx > length(frs_control_limits)
    error('Control index exceeds available control limits');
end

% Get velocity and control limit for computation
velocity = brs_velocities(opts.velocity_idx);
control_limit = brs_control_limits(opts.control_idx);

% Validate that FRS has matching velocity and control limit
frs_v_idx = find(frs_velocities == velocity);
if isempty(frs_v_idx)
    [~, frs_v_idx] = min(abs(frs_velocities - velocity));
    warning('Exact velocity match not found in FRS data. Using closest: %d', frs_velocities(frs_v_idx));
else
    frs_v_idx = frs_v_idx(1);
end

frs_c_idx = find(frs_control_limits == control_limit);
if isempty(frs_c_idx)
    [~, frs_c_idx] = min(abs(frs_control_limits - control_limit));
    warning('Exact control limit match not found in FRS data. Using closest: %g', frs_control_limits(frs_c_idx));
else
    frs_c_idx = frs_c_idx(1);
end

%% Extract BRS and FRS value functions
% Get BRS data (states that can reach the target)
brs_value_function = brs_data.all_data{opts.velocity_idx, opts.control_idx};

% Get FRS data (states that can be reached from the initial states)
frs_value_function = frs_data.all_data{frs_v_idx, frs_c_idx};

%% Compute the safe set as the intersection of BRS and FRS
% For level set functions, intersection is the maximum of the value functions
fprintf('Computing safe set as intersection of BRS and FRS...\n');
safe_set_value_function = max(brs_value_function, frs_value_function);

%% Create results directory
if opts.save_results
    % Create timestamp for folder name
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    
    % Create base folder path
    base_folder = fullfile(fileparts(brs_folder), 'safe_set_results');
    if ~exist(base_folder, 'dir')
        mkdir(base_folder);
    end
    
    % Create specific folder for this run
    if strcmp(control_type, 'dv')
        control_str = sprintf('dv%.0f', control_limit*180/pi);
    else
        control_str = sprintf('mz%d', control_limit);
    end
    
    result_folder = fullfile(base_folder, sprintf('safe_set_v%d_%s_%s', ...
        velocity, control_str, timestamp));
    
    mkdir(result_folder);
    fprintf('Created results directory: %s\n', result_folder);
    
    % Save results
    fprintf('Saving safe set results...\n');
    save(fullfile(result_folder, 'safe_set_data.mat'), ...
        'g', 'safe_set_value_function', 'brs_value_function', 'frs_value_function', ...
        'velocity', 'control_limit', 'control_type');
else
    result_folder = '';
end

%% Analyze the safe set
% Calculate the area/volume of the safe set
safe_region = (safe_set_value_function <= 0);
grid_cell_size = prod(g.dx);  % Size of each grid cell
safe_set_size = sum(safe_region(:)) * grid_cell_size;  % Total size of safe set

% Calculate the percentage of state space that is safe
full_space_size = prod(g.max - g.min);
safe_percentage = 100 * safe_set_size / full_space_size;

fprintf('\nSafe Set Analysis:\n');
if is_3d_model
    fprintf('  Volume of safe set: %.2f\n', safe_set_size);
else
    fprintf('  Area of safe set: %.2f\n', safe_set_size);
end
fprintf('  Percentage of state space that is safe: %.2f%%\n', safe_percentage);

%% Visualize results if requested
if opts.visualize
    fprintf('Visualizing safe set...\n');
    
    if is_3d_model
        % For 3D model, use the new interactive visualization
        visualize_interactive_safe_set(g, safe_set_value_function, brs_value_function, frs_value_function, velocity, control_limit, control_type);
    else
        % For 2D model, use the existing visualization
        visualize_2d_safe_set(g, safe_set_value_function, brs_value_function, frs_value_function, velocity, control_limit, control_type);
    end
    
    % Save figures if requested
    if opts.save_results
        % Save current figure
        fig = gcf;
        fig_file = fullfile(result_folder, 'safe_set_visualization.png');
        saveas(fig, fig_file);
        fprintf('Saved visualization to: %s\n', fig_file);
    end
end

fprintf('Safe set computation completed successfully.\n');
if opts.save_results
    fprintf('Results saved to: %s\n', result_folder);
end
end

%% Helper function for 2D visualization
function visualize_2d_safe_set(g, safe_set, brs, frs, velocity, control_limit, control_type)
    % Create a figure for visualization
    figure('Name', 'Safe Set Visualization (2D)', 'Position', [100, 100, 1200, 600]);
    
    % Convert grid values from radians to degrees for plotting
    xs1_deg = g.xs{1} * 180/pi;  % Convert yaw rate to degrees
    xs2_deg = g.xs{2} * 180/pi;  % Convert sideslip angle to degrees

    % Plot comparison of all three sets
    subplot(1, 2, 1);
    hold on;
    
    % Plot the sets boundaries
    [~, h_brs] = contour(xs2_deg, xs1_deg, brs, [0 0], 'LineWidth', 2, 'Color', 'b');
    [~, h_frs] = contour(xs2_deg, xs1_deg, frs, [0 0], 'LineWidth', 2, 'Color', 'r');
    [~, h_safe] = contour(xs2_deg, xs1_deg, safe_set, [0 0], 'LineWidth', 3, 'Color', 'k');
    
    % Shade the safe region
    safe_region = safe_set <= 0;
    [rows, cols] = find(safe_region);
    if ~isempty(rows)
        scatter(xs2_deg(sub2ind(size(xs2_deg), rows, cols)), ...
                xs1_deg(sub2ind(size(xs1_deg), rows, cols)), ...
                10, 'k', 'filled', 'MarkerFaceAlpha', 0.2);
    end
    
    % Add labels and legend
    xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
    ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
    
    if strcmp(control_type, 'dv')
        title_str = sprintf('Backward, Forward and Safe Sets (v = %d m/s, dvmax = %.1f°/s)', ...
                velocity, control_limit*180/pi);
    else
        title_str = sprintf('Backward, Forward and Safe Sets (v = %d m/s, Mzmax = %d N·m)', ...
                velocity, control_limit);
    end
    
    title(title_str, 'FontSize', 14, 'FontWeight', 'bold');
    legend([h_brs, h_frs, h_safe], 'Backward Reachable Set', 'Forward Reachable Set', ...
            'Safe Set', 'Location', 'best');
    
    grid on;
    axis equal;
    
    % Plot value functions as surfaces
    subplot(1, 2, 2);
    
    % Create an interpolated mesh for smoother visualization
    [X, Y] = meshgrid(linspace(min(xs2_deg(:)), max(xs2_deg(:)), 200), ...
                     linspace(min(xs1_deg(:)), max(xs1_deg(:)), 200));
    
    safe_interp = interp2(xs2_deg, xs1_deg, safe_set, X, Y, 'cubic');
    
    % Plot safe set value function surface with zero level highlighted
    surf(X, Y, safe_interp, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
    hold on;
    
    % Add the zero level contour
    [~, h_zero] = contour3(X, Y, safe_interp, [0 0], 'LineWidth', 3, 'Color', 'k');
    
    % Add colorbar and labels
    colormap(jet);
    colorbar;
    xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
    ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
    zlabel('Value', 'FontSize', 12);
    title('Safe Set Value Function', 'FontSize', 14);
    
    % Adjust view angle
    view([-30, 30]);
    grid on;
end