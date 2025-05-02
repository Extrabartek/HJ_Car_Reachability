function visualize_steering_gradients_wrapper()
% VISUALIZE_STEERING_GRADIENTS_WRAPPER Script to visualize steering gradients
% using the time each point enters the BRS, optimized for multiple slices
%
% This wrapper script precomputes gradients once and reuses them for all slices

%% Configuration Parameters - MODIFY THESE VALUES
% ---------------------------------------------------

%% Path to results folder
main_results_folder = '/home/bartosz/Documents/master_thesis/code_base/HJ_Car_Reachability/results/';
results_folder = fullfile(main_results_folder, 'steered_brs_results_20250501_103706_vx20-20_dvmax40-40');

%% Data selection
velocity_idx = 1;        % Index of velocity to use
control_idx = 1;         % Index of control limit to use

%% Visualization parameters
arrow_density = [15, 15];  % Number of arrows in each dimension [beta, gamma]
arrow_scale = 100.5;         % Scale factor for arrow size
delta_slices = [-5, 0, 5]; % Steering angle slices to visualize (in degrees)
show_arrows = false;        % Set to true to show control direction arrows
color_scheme = 'coolwarm'; % Options: 'coolwarm', 'jet', 'parula', etc.
show_boundary = true;      % Set to true to show the BRS boundary

%% Saving options
save_plots = false;      % Set to true to save visualization figures
figures_folder = 'steering_gradients'; % Subfolder for saving figures

% ---------------------------------------------------
% END OF CONFIGURATION PARAMETERS 
% ---------------------------------------------------

%% Validate paths
fprintf('\n=== Entry-Time Steering Gradient Visualization Tool ===\n');

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
    error('Steering gradient visualization is only implemented for 3D models');
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
fprintf('Extracting value function data...\n');

% Check if full time data is available
if isfield(data, 'all_data_full') && ~isempty(data.all_data_full)
    % Use full time data
    value_function_full = data.all_data_full{velocity_idx, control_idx};
    fprintf('Using full time data (%d time steps)\n', size(value_function_full, 4));
else
    % No full time data available
    error('Full time data not available. Entry-time gradient visualization requires full time data.');
end

% Extract final BRS for boundary
final_brs = value_function_full(:,:,:,end);

%% PRECOMPUTE TIME-OF-ARRIVAL AND GRADIENTS ONCE FOR ALL SLICES
fprintf('\n=== Precomputing data for all slices ===\n');

% Number of time steps
num_times = size(value_function_full, 4);

%% 1. Compute time-of-arrival function (when each point first enters the BRS)
fprintf('Computing time-of-arrival function...\n');

% Initialize with infinity (points never reaching BRS)
arrival_time_indices = inf(g.N');

% Determine when each point first enters the BRS
for t = 1:1:num_times
    % Get value function at this time
    value_t = value_function_full(:,:,:,t);
    
    % Find points that are i BRS at this time (value <= 0)
    in_brs_t = (value_t <= 0);
    
    % For points that haven't yet been assigned an arrival time, 
    % record this time if they're in the BRS
    unassigned = isinf(arrival_time_indices);
    new_points = in_brs_t & unassigned;
    
    % Assign arrival time for these points
    arrival_time_indices(new_points) = t;
    % Display progress
    if mod(t, 10) == 0 || t == 1
        fprintf('Computing entry time: processed time step %d/%d\n', num_times-t+1, num_times);
    end
end

%% 1.5. Visualize the time of BRS entry
figure('Name', 'Time of BRS entry', 'Position', [100, 100, 1200, 600]);

%% 2. Compute gradients for each time slice
fprintf('Computing gradients for all time slices...\n');
all_gradients = cell(num_times, 1);

for t = 1:num_times
    value_t = value_function_full(:,:,:,t);
    all_gradients{t} = computeGradients(g, value_t);
    
    % Display progress
    if mod(t, 10) == 0 || t == num_times
        fprintf('Computed gradients for time slice %d/%d\n', t, num_times);
    end
end

fprintf('Precomputation complete. Creating visualizations for %d slices...\n', length(delta_slices));

%% Create visualization for each specified delta slice
for i = 1:length(delta_slices)
    delta_slice = delta_slices(i);
    fprintf('\nCreating visualization for delta = %.1f degrees...\n', delta_slice);
    
    % Set up visualization options
    vis_opts = struct();
    vis_opts.density = arrow_density;
    vis_opts.scale = arrow_scale;
    vis_opts.colormap = color_scheme;
    vis_opts.showBoundary = show_boundary;
    vis_opts.showArrows = show_arrows;
    
    % Create title based on parameters
    if strcmp(control_type, 'dvmax')
        fig_title = sprintf('Entry-Time Steering Gradient (v = %d m/s, dvmax = %.1f°/s, δ = %.1f°)', ...
            velocities(velocity_idx), control_limits(control_idx)*180/pi, delta_slice);
    else
        fig_title = sprintf('Entry-Time Steering Gradient (v = %d m/s, Mzmax = %d N·m, δ = %.1f°)', ...
            velocities(velocity_idx), control_limits(control_idx), delta_slice);
    end
    
    % Create figure
    figure_handle = figure('Name', fig_title, 'Position', [100, 100, 1200, 600]);
    
    % Call visualization function with precomputed data
    visualize_steering_gradient_slice(g, final_brs, all_gradients, arrival_time_indices, ...
                                     tau, delta_slice, vis_opts);
    
    % Add title
    sgtitle(fig_title, 'FontSize', 16, 'FontWeight', 'bold');
    
    % Save figure if requested
    if save_plots
        % Create figures directory if it doesn't exist
        figures_dir = fullfile(results_folder, 'figures', figures_folder);
        if ~exist(figures_dir, 'dir')
            mkdir(figures_dir);
            fprintf('Created figures directory: %s\n', figures_dir);
        end
        
        % Create filename based on parameters
        fig_filename = sprintf('entry_time_gradient_v%d_delta%.0f.png', ...
            velocities(velocity_idx), delta_slice);
        
        % Save figure
        fig_path = fullfile(figures_dir, fig_filename);
        saveas(figure_handle, fig_path);
        fprintf('Saved visualization to: %s\n', fig_path);
    end
end

fprintf('\nAll visualizations complete!\n');
end

function visualize_steering_gradient_slice(g, final_brs, all_gradients, arrival_time_indices, ...
                                         tau, delta_slice_deg, opts)
% VISUALIZE_STEERING_GRADIENT_SLICE Visualizes a single steering angle slice
% using precomputed gradients and arrival times
%
% This function creates a visualization of the steering angle gradient component
% for a specific delta slice, using precomputed gradients and arrival times.

%% Convert delta slice from degrees to radians
delta_rad = delta_slice_deg * pi/180;

% Find the closest slice index
delta_values = g.vs{3};
[~, slice_idx] = min(abs(delta_values - delta_rad));
actual_delta = delta_values(slice_idx) * 180/pi;
fprintf('Using delta slice at %.2f degrees (closest to requested %.2f degrees)\n', ...
    actual_delta, delta_slice_deg);

%% Extract 2D slice of value function
% Extract the beta and gamma grids at the delta slice
beta_grid = g.xs{2}(:,:,slice_idx) * 180/pi;   % Convert to degrees
gamma_grid = g.xs{1}(:,:,slice_idx) * 180/pi;  % Convert to degrees

% Extract the value function slice
value_slice = squeeze(final_brs(:,:,slice_idx));

% Extract the time-of-arrival slice
arrival_time_slice = squeeze(arrival_time_indices(:,:,slice_idx));

%% Create a mask for unreachable points
reachable_mask = isfinite(arrival_time_slice);

%% Setup custom colormap for delta gradient (blue-white-red)
if strcmpi(opts.colormap, 'coolwarm')
    % Create a blue-white-red colormap (negative to positive)
    n = 256;
    cmap = zeros(n, 3);
    % Blue to white
    cmap(1:n/2, 1) = linspace(0, 1, n/2);
    cmap(1:n/2, 2) = linspace(0, 1, n/2);
    cmap(1:n/2, 3) = linspace(1, 1, n/2);
    % White to red
    cmap(n/2+1:n, 1) = linspace(1, 1, n/2);
    cmap(n/2+1:n, 2) = linspace(1, 0, n/2);
    cmap(n/2+1:n, 3) = linspace(1, 0, n/2);
    colormap(cmap);
else
    colormap(opts.colormap);
end

%% Create 2D visualization
% Split into two subplots: one for value function, one for delta gradient
subplot(1, 2, 1);
[~, h_value] = contourf(beta_grid, gamma_grid, value_slice, 20);
hold on;
[~, h_brs] = contour(beta_grid, gamma_grid, value_slice, [0, 0], 'LineWidth', 2, 'Color', 'k');
colorbar;
title(sprintf('Value Function at δ = %.1f°', actual_delta), 'FontSize', 14);
xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
grid on;

% Initialize the delta gradient field
delta_gradient = zeros(size(value_slice));
delta_gradient(~reachable_mask) = NaN; % Mark unreachable points as NaN

%% Compute entry-time gradients for each point in the slice
fprintf('Computing delta gradient field using entry times...\n');

% Number of time steps
num_times = length(all_gradients);

% Loop through all points in the slice
for i = 1:size(beta_grid, 1)
    for j = 1:size(beta_grid, 2)
        % Skip points that never enter the BRS
        if ~reachable_mask(i, j)
            continue;
        end
        
        % Get the entry time index for this point
        entry_idx = arrival_time_slice(i, j);
        
        % Ensure valid index
        entry_idx = max(1, min(num_times, round(entry_idx)));
        
        % Get the gradient at this entry time
        entry_gradients = all_gradients{entry_idx};
        
        % Create state vector [gamma, beta, delta]
        x = [gamma_grid(i, j)*pi/180, beta_grid(i, j)*pi/180, delta_rad]';
        
        % Evaluate gradient at this point and time
        grad = eval_u(g, entry_gradients, x);
        
        % Store the delta gradient component
        delta_gradient(i, j) = grad(3);
    end
end

%% Plot the delta gradient component based on entry time
subplot(1, 2, 2);
[~, h_grad] = contourf(beta_grid, gamma_grid, delta_gradient, 20);
hold on;
[~, h_brs2] = contour(beta_grid, gamma_grid, value_slice, [0, 0], 'LineWidth', 2, 'Color', 'k');
c = colorbar;
title(c, '∂V/∂δ');
max_grad = max(abs(delta_gradient(:)), [], 'omitnan');
clim([-max_grad, max_grad]);  % Symmetric colorbar
title(sprintf('Entry-Time Steering Gradient (∂V/∂δ) at δ = %.1f°', actual_delta), 'FontSize', 14);
xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
grid on;

%% Add control arrows if requested
if opts.showArrows
    % Sample points for arrows
    [beta_samples, gamma_samples] = meshgrid(...
        linspace(min(beta_grid(:)), max(beta_grid(:)), opts.density(1)),...
        linspace(min(gamma_grid(:)), max(gamma_grid(:)), opts.density(2)));
    
    % Evaluate delta gradient at sample points based on entry time
    delta_grad_samples = zeros(size(beta_samples));
    in_brs_mask = false(size(beta_samples));
    
    for i = 1:numel(beta_samples)
        % Convert to radians for evaluation
        beta_rad = beta_samples(i) * pi/180;
        gamma_rad = gamma_samples(i) * pi/180;
        
        % Create state vector [gamma, beta, delta]
        x = [gamma_rad, beta_rad, delta_rad]';
        
        % Check if point is in BRS
        value = eval_u(g, final_brs, x);
        if value > 0
            % Point is outside the BRS, skip
            continue;
        end
        
        % Get entry time for this point
        entry_time_idx = eval_u(g, arrival_time_indices, x);
        
        % Skip points that never enter BRS
        if isinf(entry_time_idx) || isnan(entry_time_idx)
            continue;
        end
        
        % Convert to valid index
        entry_time_idx = max(1, min(num_times, round(entry_time_idx)));
        
        % Get gradient at entry time
        entry_gradients = all_gradients{entry_time_idx};
        
        % Evaluate gradient at this point
        grad = eval_u(g, entry_gradients, x);
        
        % Store delta gradient component
        delta_grad_samples(i) = grad(3);
        
        % Mark as in BRS
        in_brs_mask(i) = true;
    end
    
    % Create arrows showing control direction
    % For steering control with uMode='min': negative gradient → positive control
    % Plot in the gradient subplot
    subplot(1, 2, 2);
    
    % Filter out points outside the BRS
    beta_valid = beta_samples(in_brs_mask);
    gamma_valid = gamma_samples(in_brs_mask);
    delta_grad_valid = delta_grad_samples(in_brs_mask);
    
    if ~isempty(beta_valid)
        % Scale arrow size based on gradient magnitude
        max_grad = max(abs(delta_grad_valid));
        if max_grad > 0
            norm_grad = delta_grad_valid / max_grad * opts.scale;
            
            % Create arrows with changing colors based on sign
            % Positive gradient (red) = negative control = steering left
            % Negative gradient (blue) = positive control = steering right
            quiver(beta_valid, gamma_valid, ...
                  zeros(size(beta_valid)), -norm_grad, ...
                  0, 'LineWidth', 1.5);
            
            % Add info text about control direction
            text(min(beta_grid(:))+1, max(gamma_grid(:))-5, ...
                 'Blue = Steer Right (δ̇ > 0)', 'Color', 'blue', 'FontWeight', 'bold');
            text(min(beta_grid(:))+1, max(gamma_grid(:))-15, ...
                 'Red = Steer Left (δ̇ < 0)', 'Color', 'red', 'FontWeight', 'bold');
        end
    end
end

fprintf('Visualization for delta = %.1f degrees complete.\n', actual_delta);
end