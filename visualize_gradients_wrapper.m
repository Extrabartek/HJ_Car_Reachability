function visualize_gradients_wrapper()
% VISUALIZE_GRADIENTS_WRAPPER Script to visualize value function gradients
%
% This wrapper script allows you to easily configure and run gradient 
% visualizations for reachability analysis results.
%
% To use:
% 1. Modify the configuration parameters in the section below
% 2. Run the script
%
% The script will load the data and generate the visualizations as specified.

%% Configuration Parameters - MODIFY THESE VALUES
% ---------------------------------------------------

%% Path to results folder
main_results_folder = '/home/bartosz/Documents/master_thesis/code_base/HJ_Car_Reachability/results/';
results_folder = fullfile(main_results_folder, 'steered_brs_results_20250429_142622_vx30-30_dvmax20-20');

%% Data selection
velocity_idx = 1;        % Index of velocity to use
control_idx = 1;         % Index of control limit to use
use_target_set = false;  % Set to true to visualize gradients of target set instead of BRS

%% Visualization parameters
sampling_density = [15, 15, 10]; % Number of arrows in each dimension [gamma, beta, delta]
                             % Use lower numbers (8-15) for memory efficiency
arrow_scale = 2;       % Scale factor for arrow size
arrow_color = 'r';       % Color for arrows: 'r', 'g', 'b', or [R G B]
normalize_arrows = true; % Set to true to show only direction (not magnitude)
show_boundary = true;    % Set to true to show the BRS boundary

%% Filtering options
value_threshold = 10;   % Only show gradients where |value function| <= threshold
                         % Set to inf to show all gradients
                         
% Slice options
slice_enabled = false;    % Set to true to only show gradients on a specific slice
slice_dimension = 3;     % Dimension to slice: 1 (gamma), 2 (beta), or 3 (delta)
slice_value = 0;         % Value for slice in degrees (will be converted to radians)

%% Saving options
save_plots = false;      % Set to true to save visualization figures
figure_filename = 'gradient_visualization.png'; % Filename for saved figure

% ---------------------------------------------------
% END OF CONFIGURATION PARAMETERS 
% ---------------------------------------------------

%% Validate paths
fprintf('\n=== Gradient Visualization Tool ===\n');

if ~exist(results_folder, 'dir')
    error('Results folder not found: %s', results_folder);
end

% Find and load combined results file
combined_file = fullfile(results_folder, 'brs_combined_results.mat');
if ~exist(combined_file, 'file')
    % Try alternate name
    combined_file = fullfile(results_folder, 'frs_combined_results.mat');
    if ~exist(combined_file, 'file')
        error('Combined results file not found in the specified folder');
    end
end

%% Load the data
fprintf('Loading data from: %s\n', combined_file);
data = load(combined_file);

% Extract grid information
g = data.g;

% Check if the model is 3D (steered bicycle)
is_3d_model = (length(g.N) == 3);
if ~is_3d_model
    error('Gradient visualization is currently only implemented for 3D models');
end

% Extract velocity and control information
velocities = data.velocities;
if isfield(data, 'dvmax_values')
    control_limits = data.dvmax_values;
    control_type = 'dvmax';
elseif isfield(data, 'mzmax_values')
    control_limits = data.mzmax_values;
    control_type = 'mzmax';
else
    error('Could not determine control type from data');
end

% Extract time vector
tau = data.tau;

% Validate indices
if velocity_idx > length(velocities)
    warning('Velocity index exceeds available velocities. Using index 1.');
    velocity_idx = 1;
end

if control_idx > length(control_limits)
    warning('Control index exceeds available control limits. Using index 1.');
    control_idx = 1;
end

fprintf('Using velocity = %d m/s, %s = %g\n', velocities(velocity_idx), ...
    control_type, control_limits(control_idx));

% Extract the value function data
if use_target_set
    fprintf('Using target set for gradient visualization\n');
    value_function = data.data0;
    % Since target set doesn't change with time, we don't need full time data
    value_function_full = value_function;
else
    fprintf('Using BRS/FRS for gradient visualization\n');
    
    % Check if full time data is available
    if isfield(data, 'all_data_full') && ~isempty(data.all_data_full)
        % Use full time data
        value_function_full = data.all_data_full{velocity_idx, control_idx};
        fprintf('Using full time data (%d time steps)\n', size(value_function_full, 4));
    else
        % No full time data available
        warning('Full time data not available. Using single time slice only.');
        value_function_full = data.all_data{velocity_idx, control_idx};
    end
end

%% Prepare visualization options
vis_opts = struct();
vis_opts.density = sampling_density;
vis_opts.scale = arrow_scale;
vis_opts.normalize = normalize_arrows;
vis_opts.threshold = value_threshold;
vis_opts.arrowColor = arrow_color;
vis_opts.showBoundary = show_boundary;

% Set up slice if enabled
if slice_enabled
    % Convert slice value from degrees to radians for internal use
    slice_rad = slice_value * pi/180;
    
    % Find the closest index corresponding to this value
    dim_values = g.vs{slice_dimension};
    [~, slice_idx] = min(abs(dim_values - slice_rad));
    
    vis_opts.slice = slice_idx;
    vis_opts.sliceDim = slice_dimension;
    
    fprintf('Using slice in dimension %d at %.1f degrees (index %d)\n', ...
        slice_dimension, slice_value, slice_idx);
end

%% Call the visualization function
fprintf('Generating gradient visualization...\n');

% Create title based on parameters
if strcmp(control_type, 'dvmax')
    fig_title = sprintf('Gradient Visualization (v = %d m/s, dvmax = %.1f°/s)', ...
        velocities(velocity_idx), control_limits(control_idx)*180/pi);
else
    fig_title = sprintf('Gradient Visualization (v = %d m/s, Mzmax = %d N·m)', ...
        velocities(velocity_idx), control_limits(control_idx));
end

% Create figure and set title
figure_handle = figure('Name', 'Gradient Visualization', 'Position', [100, 100, 900, 700]);
sgtitle(fig_title, 'FontSize', 16, 'FontWeight', 'bold');

% Call visualization function with our options
visualize_gradients_3d(g, value_function_full, tau, vis_opts);

%% Save figure if requested
if save_plots
    % Create figures directory if it doesn't exist
    figures_dir = fullfile(results_folder, 'figures');
    if ~exist(figures_dir, 'dir')
        mkdir(figures_dir);
        fprintf('Created figures directory: %s\n', figures_dir);
    end
    
    % Save figure
    fig_path = fullfile(figures_dir, figure_filename);
    saveas(figure_handle, fig_path);
    fprintf('Saved visualization to: %s\n', fig_path);
end

fprintf('Gradient visualization complete!\n');

end