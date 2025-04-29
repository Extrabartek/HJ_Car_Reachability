function reachability_visualization_tool()
% REACHABILITY_VISUALIZATION_TOOL A unified tool for visualization of reachability analysis results
%
% This script provides a single interface for:
% 1. Loading and visualizing existing reachability computation results
% 2. Computing optimal trajectories using different methods 
% 3. Visualizing trajectories in state space and physical space
% 4. Comparing different methods and parameters
%
% The tool works with the new computeOptimizedTrajectory function for enhanced
% trajectory computation, while maintaining compatibility with older functions.
%
% Usage:
%   Simply run this script and follow the interactive menu prompts
%   or modify the configuration parameters at the beginning

%% Configuration Parameters - MODIFY THESE VALUES
% ---------------------------------------------------

%% Path to results folder
main_results_folder = '/home/bartosz/Documents/master_thesis/code_base/HJ_Car_Reachability/results/';  % Change this to your results directory

%% Reachability result selection
% Path to the reachability results folder
brs_folder = fullfile(main_results_folder, 'steered_brs_results_20250429_142622_vx30-30_dvmax20-20');

% Optional: Path to FRS results folder (for safety constraints) - leave empty to disable
frs_folder = fullfile(main_results_folder, 'steered_frs_results_20250429_153753_vx20-20_dvmax60-60');  % Set to an actual folder path to enable FRS constraints

%% Visualization options
visualize_reachable_sets = true;  % Set to true to visualize reachable sets
visualize_trajectory = true;      % Set to true to visualize trajectory
save_plots = false;               % Set to true to save visualization figures
save_video = false;               % Set to true to save trajectory video
video_file = 'trajectory.avi';    % Video filename for trajectory visualization
plot_types = {};  % Visualization types for reachable sets

%% Trajectory computation options
compute_new_trajectory = true;   % Set to false to load a previously saved trajectory
trajectory_file = 'trajectory_data.mat';  % For loading/saving trajectory data

% Initial state for trajectory computation
% Format: [gamma; beta; delta] (yaw rate, sideslip angle, steering angle) in radians
% Example: 10 deg/s yaw rate, 5 deg sideslip, 3 deg steering
xinit = [deg2rad(0); deg2rad(-5); deg2rad(0)];

% Trajectory computation method - options: 'arrival', 'gradient', or 'legacy'
% 'arrival'  - Uses time-of-arrival function for guidance (fastest)
% 'gradient' - Uses pre-computed gradients for all time slices (more accurate)
% 'legacy'   - Uses the older computeOptTraj or compute_trajectory_steered_from_folders
trajectory_method = 'gradient';  

% Parameters for trajectory computation
velocity_idx = 1;               % Index of velocity to use from BRS data
control_idx = 1;                % Index of control limit to use (dv_max or Mz)
max_time = 10.0;                 % Maximum trajectory time (seconds)
use_frs_constraint = false;     % Use FRS for safety constraints
frs_weight = 0.5;               % Weight for FRS constraints (0-1)

%% Vehicle/car visualization parameters
car_length = 4.5;               % Car length in meters
car_width = 1.8;                % Car width in meters 
wheel_base = 2.7;               % Distance between axles in meters
grid_size = 20;                 % Size of the grid in the car view

% ---------------------------------------------------
% END OF CONFIGURATION PARAMETERS
% ---------------------------------------------------

%% Parameter validation and setup
fprintf('\n=== Reachability Visualization Tool ===\n');

% Validate paths
if ~exist(brs_folder, 'dir')
    error('BRS folder not found: %s', brs_folder);
end

combined_file = fullfile(brs_folder, 'brs_combined_results.mat');
if ~exist(combined_file, 'file')
    error('Combined results file not found: %s', combined_file);
end

% Validate FRS folder if provided
if ~isempty(frs_folder) && ~exist(frs_folder, 'dir')
    warning('FRS folder not found: %s. FRS constraints will be disabled.', frs_folder);
    frs_folder = '';
    use_frs_constraint = false;
end

% Enable FRS constraints if FRS folder is provided
if ~isempty(frs_folder)
    use_frs_constraint = true;
    fprintf('FRS constraints enabled using data from: %s\n', frs_folder);
end

%% Load BRS data
fprintf('Loading BRS data from: %s\n', brs_folder);
brs_data = load(combined_file);

% Extract key information from BRS data
g = brs_data.g;
data0 = brs_data.data0;  % Target set
velocities = brs_data.velocities;
tau = brs_data.tau;  % Time vector
base_params = brs_data.base_params;

% Determine model type (steering or yaw moment)
is_steered_model = length(g.N) == 3;  % 3D grid indicates steered model
if is_steered_model
    fprintf('Detected 3D steered bicycle model\n');
    if isfield(brs_data, 'dvmax_values')
        control_limits = brs_data.dvmax_values;
        control_type = 'dv';
    else
        error('Could not find dvmax_values in BRS data for steered model');
    end
else
    fprintf('Detected 2D standard bicycle model with yaw moment control\n');
    if isfield(brs_data, 'mzmax_values')
        control_limits = brs_data.mzmax_values;
        control_type = 'mz';
    else
        error('Could not find mzmax_values in BRS data for standard model');
    end
end

% Validate indices
if velocity_idx > length(velocities)
    warning('Velocity index exceeds available velocities. Using index 1.');
    velocity_idx = 1;
end

if control_idx > length(control_limits)
    warning('Control index exceeds available control limits. Using index 1.');
    control_idx = 1;
end

% Extract BRS data for selected velocity and control limit
data_brs = brs_data.all_data{velocity_idx, control_idx};

% Extract full time data if available
if isfield(brs_data, 'all_data_full')
    data_brs_full = brs_data.all_data_full{velocity_idx, control_idx};
else
    warning('Full-time BRS data not available. Some optimized methods may not work correctly.');
    data_brs_full = data_brs;  % Use single time slice as fallback
end

% Load FRS data if enabled
data_frs = [];
if use_frs_constraint
    frs_combined_file = fullfile(frs_folder, 'frs_combined_results.mat');
    if ~exist(frs_combined_file, 'file')
        % Try alternate filename
        frs_combined_file = fullfile(frs_folder, 'brs_combined_results.mat');
        if ~exist(frs_combined_file, 'file')
            warning('FRS results file not found. FRS constraints will be disabled.');
            use_frs_constraint = false;
        else
            fprintf('Loading FRS data from: %s\n', frs_combined_file);
            frs_data = load(frs_combined_file);
            
            % Find matching velocity and control limit in FRS data
            v_idx = find(frs_data.velocities == velocities(velocity_idx), 1);
            if isempty(v_idx)
                % Find closest
                [~, v_idx] = min(abs(frs_data.velocities - velocities(velocity_idx)));
            end
            
            if strcmp(control_type, 'dv')
                c_idx = find(frs_data.dvmax_values == control_limits(control_idx), 1);
                if isempty(c_idx)
                    [~, c_idx] = min(abs(frs_data.dvmax_values - control_limits(control_idx)));
                end
            else
                c_idx = find(frs_data.mzmax_values == control_limits(control_idx), 1);
                if isempty(c_idx)
                    [~, c_idx] = min(abs(frs_data.mzmax_values - control_limits(control_idx)));
                end
            end
            
            data_frs = frs_data.all_data{v_idx, c_idx};
        end
    end
end

%% Compute or load arrival time function
% The arrival time function improves trajectory computation
has_arrival_time = false;
arrival_time = [];

if exist('compute_arrival_time', 'file') && ~isempty(data_brs_full) && strcmp(trajectory_method, 'arrival')
    fprintf('Computing time-of-arrival function for improved trajectory planning...\n');
    try
        arrival_time = compute_arrival_time(data_brs_full, tau);
        has_arrival_time = true;
        fprintf('Time-of-arrival function computation successful.\n');
    catch err
        warning('Failed to compute time-of-arrival function: %s\n', err.message);
        fprintf('Falling back to gradient method.\n');
        trajectory_method = 'gradient';
    end
end

%% Visualize reachable sets if requested
if visualize_reachable_sets
    fprintf('Visualizing reachable sets...\n');
    
    % Create visualization options
    viz_options = struct();
    viz_options.plotType = plot_types;
    viz_options.saveFigs = save_plots;
    viz_options.figFormat = 'png';
    viz_options.controlIdx = control_idx;
    viz_options.velocityIdx = velocity_idx;
    
    % Add model-specific options
    if is_steered_model
        % For 3D steered bicycle model
        viz_options.deltaSlices = [-0.1, 0, 0.1]; % Default slices
    end
    
    try
        % Check if unified visualization function exists
        if exist('visualize_reachability_results', 'file')
            visualize_reachability_results(brs_folder, viz_options);
        else
            warning('visualize_reachability_results function not found. Skipping reachable set visualization.');
        end
    catch err
        warning('Error during reachable set visualization: %s', err.message);
    end
end

%% Compute or load trajectory
if visualize_trajectory
    % Set up dynamic system for trajectory computation
    % Update vehicle parameters with selected velocity and control limits
    params = base_params;
    params(2) = velocities(velocity_idx);  % Set velocity
    
    if is_steered_model
        % For steered bicycle model
        if strcmp(control_type, 'dv')
            params(7) = control_limits(control_idx);  % Set dv_max
            fprintf('Using steered bicycle model with Vx = %d m/s, dv_max = %.2f rad/s (%.1f deg/s)\n', ...
                velocities(velocity_idx), control_limits(control_idx), control_limits(control_idx)*180/pi);
            
            % Check initial state dimension
            if length(xinit) ~= 3
                error('Initial state must have 3 dimensions [gamma; beta; delta] for steered model');
            end
            
            % Create the dynamic system object
            dynSys = NonlinearBicycleSteered(xinit, params);
        else
            error('Unexpected control type %s for steered model', control_type);
        end
    else
        % For standard bicycle model
        if strcmp(control_type, 'mz')
            params(7) = control_limits(control_idx);   % Set Mzmax
            params(8) = -control_limits(control_idx);  % Set Mzmin
            fprintf('Using standard bicycle model with Vx = %d m/s, Mzmax = %d N·m\n', ...
                velocities(velocity_idx), control_limits(control_idx));
            
            % Check initial state dimension
            if length(xinit) ~= 2
                error('Initial state must have 2 dimensions [gamma; beta] for standard model');
            end
            
            % Create the dynamic system object
            dynSys = NonlinearBicycle(xinit, params);
        else
            error('Unexpected control type %s for standard model', control_type);
        end
    end
    
    % Compute or load trajectory
    if compute_new_trajectory
        fprintf('Computing trajectory for initial state: ');
        if is_steered_model
            fprintf('[%.1f°/s, %.1f°, %.1f°]...\n', ...
                xinit(1)*180/pi, xinit(2)*180/pi, xinit(3)*180/pi);
        else
            fprintf('[%.1f°/s, %.1f°]...\n', ...
                xinit(1)*180/pi, xinit(2)*180/pi);
        end
        
        % Check if initial state is in BRS
        initial_value = eval_u(g, data_brs, xinit);
        if initial_value > 0
            error('Initial state is not in the BRS (value = %.4f). Try a different initial state.', initial_value);
        end
        
        % Select trajectory computation method
        if strcmp(trajectory_method, 'legacy')
            % Use legacy computation methods
            if is_steered_model
                fprintf('Using legacy steered trajectory computation method...\n');
                try
                    [traj, traj_tau, traj_u, metrics] = compute_trajectory_steered_from_folders(brs_folder, xinit, ...
                        'velocityIdx', velocity_idx, ...
                        'dvMaxIdx', control_idx, ...
                        'visualize', false, ...
                        'maxTime', max_time, ...
                        'useFRS', use_frs_constraint, ...
                        'frs_folder', frs_folder, ...
                        'frsWeight', frs_weight);
                catch err
                    error('Error in legacy trajectory computation: %s', err.message);
                end
            else
                fprintf('Using legacy standard trajectory computation method...\n');
                try
                    % Assuming a function similar to compute_trajectory_from_folders for standard model
                    [traj, traj_tau, traj_u, metrics] = compute_trajectory_from_folders(brs_folder, xinit, ...
                        'velocityIdx', velocity_idx, ...
                        'mzMaxIdx', control_idx, ...
                        'visualize', false, ...
                        'maxTime', max_time, ...
                        'useFRS', use_frs_constraint, ...
                        'frs_folder', frs_folder, ...
                        'frsWeight', frs_weight);
                catch err
                    error('Error in legacy trajectory computation: %s', err.message);
                end
            end
        else
            % Use the new optimized trajectory computation
            fprintf('Using optimized trajectory computation with method: %s\n', trajectory_method);
            
            % Check if computeOptimizedTrajectory function exists
            if ~exist('computeOptimizedTrajectory', 'file')
                error('computeOptimizedTrajectory function not found. Please add it to your MATLAB path.');
            end
            
            % Setup options for optimized computation
            opt_options = struct();
            opt_options.method = trajectory_method;
            opt_options.uMode = 'min';  % For target reaching
            opt_options.maxTime = max_time;
            opt_options.dt = tau(2) - tau(1);  % Use same time step as BRS computation
            opt_options.visualize = false;
            opt_options.finalSet = data0;  % Use target set from BRS
            
            % Add arrival time if available
            if has_arrival_time
                opt_options.arrivalTime = arrival_time;
            end
            
            % Add FRS constraints if enabled
            if use_frs_constraint && ~isempty(data_frs)
                opt_options.useFRS = true;
                opt_options.data_frs = data_frs;
                opt_options.frsWeight = frs_weight;
            end
            
            try
                % Compute trajectory using optimized method
                [traj, traj_tau, traj_u, metrics] = computeOptimizedTrajectory(g, data_brs_full, tau, xinit, dynSys, opt_options);
            catch err
                error('Error in optimized trajectory computation: %s', err.message);
            end
        end
        
        % Save computed trajectory
        save(trajectory_file, 'traj', 'traj_tau', 'traj_u', 'metrics', 'xinit', 'brs_folder', ...
            'trajectory_method', 'params', 'velocity_idx', 'control_idx');
        fprintf('Trajectory computed and saved to %s\n', trajectory_file);
    else
        % Load a previously computed trajectory
        if exist(trajectory_file, 'file')
            fprintf('Loading trajectory from %s\n', trajectory_file);
            load(trajectory_file, 'traj', 'traj_tau', 'traj_u', 'metrics', 'xinit');
            
            % Check if trajectory file contains the required data
            if ~exist('traj', 'var') || ~exist('traj_tau', 'var')
                error('Trajectory file does not contain required data');
            end
        else
            error('Trajectory file not found: %s', trajectory_file);
        end
    end
    
    %% Display trajectory metrics
    fprintf('\n=== Trajectory Metrics ===\n');
    fprintf('Computation Method: %s\n', trajectory_method);
    
    if isfield(metrics, 'time_to_target')
        fprintf('Time to target: %.2f seconds\n', metrics.time_to_target);
    end
    
    if isfield(metrics, 'final_set_value')
        if metrics.final_set_value <= 0
            fprintf('Target reached! (final value: %.4f)\n', metrics.final_set_value);
        else
            fprintf('Target NOT reached (final value: %.4f)\n', metrics.final_set_value);
        end
    end
    
    if isfield(metrics, 'control_energy')
        fprintf('Control energy: %.2e\n', metrics.control_energy);
    end
    
    if isfield(metrics, 'in_frs') && use_frs_constraint
        if metrics.in_frs
            fprintf('Trajectory stays within FRS bounds (safe)\n');
        else
            fprintf('WARNING: Trajectory violates FRS bounds at %.2f seconds\n', metrics.frs_violation_time);
        end
    end
    
    %% Visualize trajectory
    fprintf('\nVisualizing trajectory...\n');
    
    try
        % First visualize in state space
        figure('Name', 'Trajectory in State Space');
        
        if is_steered_model
            % For 3D model, show slices at the steering angle
            delta_values = g.xs{3}(1, 1, :);
            center_idx = ceil(length(delta_values)/2);
            
            % Find the index closest to the initial steering angle
            [~, delta_idx] = min(abs(delta_values - xinit(3)));
            
            % Extract 2D slices
            data_slice = squeeze(data_brs(:,:,delta_idx));
            target_slice = squeeze(data0(:,:,center_idx));
            
            % Plot the slice
            xs1_deg = g.xs{1}(:,:,1) * 180/pi;  % yaw rate
            xs2_deg = g.xs{2}(:,:,1) * 180/pi;  % sideslip angle
            
            hold on;
            [~, h1] = contour(xs2_deg, xs1_deg, data_slice, [0, 0], 'LineWidth', 2, 'Color', 'k');
            [~, h2] = contour(xs2_deg, xs1_deg, target_slice, [0, 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
            
            % Convert trajectory to degrees for plotting
            traj_deg = traj * 180/pi;
            
            % Plot trajectory in state space (sideslip vs yaw rate)
            h3 = plot(traj_deg(2,:), traj_deg(1,:), 'b-', 'LineWidth', 2);
            h4 = plot(traj_deg(2,1), traj_deg(1,1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
            h5 = plot(traj_deg(2,end), traj_deg(1,end), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
            
            xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
            ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
            title(sprintf('Trajectory in State Space (δ = %.1f°)', delta_values(delta_idx)*180/pi), 'FontSize', 14);
            legend([h1, h2, h3, h4, h5], 'BRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State', 'Location', 'best');
            grid on;
            
            % Also create a figure showing the steering angle over time
            figure('Name', 'Steering Angle vs Time');
            plot(traj_tau, traj_deg(3,:), 'r-', 'LineWidth', 2);
            xlabel('Time (s)', 'FontSize', 12);
            ylabel('Steering Angle (degrees)', 'FontSize', 12);
            title('Steering Angle vs Time', 'FontSize', 14);
            grid on;
            
        else
            % For 2D model
            hold on;
            [~, h1] = contour(g.xs{2}*180/pi, g.xs{1}*180/pi, data_brs, [0, 0], 'LineWidth', 2, 'Color', 'k');
            [~, h2] = contour(g.xs{2}*180/pi, g.xs{1}*180/pi, data0, [0, 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
            
            % Convert trajectory to degrees for plotting
            traj_deg = traj * 180/pi;
            
            % Plot trajectory
            h3 = plot(traj_deg(2,:), traj_deg(1,:), 'b-', 'LineWidth', 2);
            h4 = plot(traj_deg(2,1), traj_deg(1,1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
            h5 = plot(traj_deg(2,end), traj_deg(1,end), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
            
            xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
            ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
            title('Trajectory in State Space', 'FontSize', 14);
            legend([h1, h2, h3, h4, h5], 'BRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State', 'Location', 'best');
            grid on;
        end
        
        % Also visualize control inputs
        figure('Name', 'Control Inputs');
        plot(traj_tau(1:end-1), traj_u, 'r-', 'LineWidth', 2);
        xlabel('Time (s)', 'FontSize', 12);
        
        if is_steered_model
            ylabel('Steering Rate (rad/s)', 'FontSize', 12);
            title('Steering Rate Control Input', 'FontSize', 14);
        else
            ylabel('Yaw Moment (N·m)', 'FontSize', 12);
            title('Yaw Moment Control Input', 'FontSize', 14);
        end
        grid on;
        
        % Visualize car trajectory if possible
        if exist('visualizeCarTrajectory', 'file')
            fprintf('Visualizing physical car trajectory...\n');
            
            % Get the velocity for this simulation
            vx = velocities(velocity_idx);
            
            try
                visualizeCarTrajectory(traj, traj_tau, g, data_brs, data0, vx, ...
                    'x0', 0, 'y0', 0, 'psi0', 0, ...
                    'saveVideo', save_video, ...
                    'videoFile', video_file, ...
                    'carLength', car_length, ...
                    'carWidth', car_width, ...
                    'wheelBase', wheel_base, ...
                    'gridSize', grid_size, ...
                    'playSpeed', 1.0);
                
                if save_video
                    fprintf('Car trajectory visualization saved to: %s\n', video_file);
                end
            catch err
                warning('Error in car trajectory visualization: %s', err.message);
                
                % Try again without video if the error is video-related
                if save_video && (contains(err.message, 'VideoWriter') || contains(err.message, 'video'))
                    fprintf('Attempting visualization without video recording...\n');
                    try
                        visualizeCarTrajectory(traj, traj_tau, g, data_brs, data0, vx, ...
                            'x0', 0, 'y0', 0, 'psi0', 0, ...
                            'saveVideo', false, ...
                            'carLength', car_length, ...
                            'carWidth', car_width, ...
                            'wheelBase', wheel_base, ...
                            'gridSize', grid_size, ...
                            'playSpeed', 1.0);
                        fprintf('Car trajectory visualization completed without video recording.\n');
                    catch inner_err
                        warning('Visualization failed: %s', inner_err.message);
                    end
                end
            end
        else
            warning('visualizeCarTrajectory function not found. Skipping physical car visualization.');
        end
        
    catch err
        warning('Error during trajectory visualization: %s', err.message);
    end
end

fprintf('\nReachability visualization tool execution complete.\n');
end