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
% Supported dynamic systems:
% - Bicycle model (Standard and Steered variants)
% - Double Integrator
% - Dubins Car
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
brs_folder = fullfile(main_results_folder, 'dubinscar_brs_results_20250507_142451_v5_turn40-40');

% Optional: Path to FRS results folder - required for FRS trajectory visualization
frs_folder = fullfile(main_results_folder, 'steered_frs_results_20250501_103930_vx20-20_dvmax40-40');

%% Trajectory type selection
% Set to 'brs' for backward reachable trajectories (going to target)
% Set to 'frs' for forward reachable trajectories (escaping from target)
trajectory_type = 'brs';  % Choose 'brs' or 'frs'

%% Visualization options
visualize_reachable_sets = true;  % Set to true to visualize reachable sets
visualize_trajectory = true;      % Set to true to visualize trajectory
save_plots = false;               % Set to true to save visualization figures
save_video = false;               % Set to true to save trajectory video
video_file = 'trajectory1.avi';    % Video filename for trajectory visualization
plot_types = {};  % Visualization types for reachable sets

%% Trajectory computation options
compute_new_trajectory = true;   % Set to false to load a previously saved trajectory
trajectory_file = 'trajectory_data.mat';  % For loading/saving trajectory data

% Initial state for trajectory computation - format depends on system type:
% - Bicycle Model:    [gamma; beta; delta] (yaw rate, sideslip angle, steering angle) in radians
% - Double Integrator: [position; velocity]
% - Dubins Car:       [x; y; theta] (position and heading) in meters and radians
xinit = [-5; 0; 0];

% Trajectory computation method - options: 'arrival', 'gradient', or 'legacy'
% 'arrival'  - Uses time-of-arrival function for guidance (fastest)
% 'gradient' - Uses pre-computed gradients for all time slices (more accurate)
% 'legacy'   - Uses the older computeOptTraj or compute_trajectory_steered_from_folders
trajectory_method = 'gradient';  

% Parameters for trajectory computation
velocity_idx = 1;               % Index of velocity to use from data (for bicycle models)
control_idx = 1;                % Index of control limit to use 
max_time = 5.5;                % Maximum trajectory time (seconds)
use_frs_constraint = false;     % Use FRS for safety constraints (for BRS trajectories only)
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

% Validate trajectory type
if ~ismember(trajectory_type, {'brs', 'frs'})
    error('Invalid trajectory_type. Must be either "brs" or "frs"');
end

% Validate paths
if strcmp(trajectory_type, 'brs') && ~exist(brs_folder, 'dir')
    error('BRS folder not found: %s', brs_folder);
end

if strcmp(trajectory_type, 'frs') && ~exist(frs_folder, 'dir')
    error('FRS folder not found: %s', frs_folder);
end

% Set the active folder based on trajectory type
if strcmp(trajectory_type, 'brs')
    active_folder = brs_folder;
    combined_file = fullfile(active_folder, 'brs_combined_results.mat');
else
    active_folder = frs_folder;
    combined_file = fullfile(active_folder, 'frs_combined_results.mat');
    
    % Try alternate filename if the main one doesn't exist
    if ~exist(combined_file, 'file')
        combined_file = fullfile(active_folder, 'brs_combined_results.mat');
    end
end

if ~exist(combined_file, 'file')
    error('Combined results file not found: %s', combined_file);
end

% Validate FRS folder if provided (for BRS trajectory with FRS constraints)
if strcmp(trajectory_type, 'brs') && use_frs_constraint && (~isempty(frs_folder) && ~exist(frs_folder, 'dir'))
    warning('FRS folder not found: %s. FRS constraints will be disabled.', frs_folder);
    frs_folder = '';
    use_frs_constraint = false;
end

%% Load data
fprintf('Loading %s data from: %s\n', upper(trajectory_type), active_folder);
active_data = load(combined_file);

% Extract key information
g = active_data.g;
data0 = active_data.data0;  % Target set
tau = active_data.tau;  % Time vector
base_params = active_data.base_params;

%% Determine model type (bicycle, doubleInt, or dubinsCar)
model_type = '';
if isfield(active_data, 'modelType')
    % If modelType is directly stored in the data (preferred)
    model_type = active_data.modelType;
elseif isfield(active_data, 'sim_params') && isfield(active_data.sim_params, 'modelType')
    % Check if it's in the sim_params structure
    model_type = active_data.sim_params.modelType;
else
    % Try to infer model type from available fields and structure
    if length(g.N) == 3 && isfield(active_data, 'dvmax_values')
        model_type = 'bicycle';
        is_steered_model = true;
        control_type = 'dv';
    elseif length(g.N) == 2 && isfield(active_data, 'mzmax_values')
        model_type = 'bicycle';
        is_steered_model = false;
        control_type = 'mz';
    elseif isfield(active_data, 'acceleration_limits')
        model_type = 'doubleInt';
    elseif isfield(active_data, 'turning_limits') || isfield(active_data, 'speed')
        model_type = 'dubinsCar';
    else
        % Default to bicycle based on dimensions if we can't determine
        is_steered_model = length(g.N) == 3;
        model_type = 'bicycle';
        control_type = is_steered_model;
        warning('Could not determine model type explicitly; inferring as %s');
    end
end

%% Process model-specific parameters
if strcmp(model_type, 'bicycle')
    % Extract bicycle model parameters
    velocities = active_data.velocities;
    is_steered_model = length(g.N) == 3;
    
    if is_steered_model
        if isfield(active_data, 'dvmax_values')
            control_limits = active_data.dvmax_values;
            control_type = 'dv';
        else
            error('Could not find dvmax_values in data for steered model');
        end
    else
        if isfield(active_data, 'mzmax_values')
            control_limits = active_data.mzmax_values;
            control_type = 'mz';
        else
            error('Could not find mzmax_values in data for standard model');
        end
    end
    
    fprintf('Model detected: bicycle\n');
    
elseif strcmp(model_type, 'doubleInt')
    % Extract Double Integrator parameters
    if isfield(active_data, 'acceleration_limits')
        control_limits = active_data.acceleration_limits;
    elseif isfield(active_data, 'sim_params') && isfield(active_data.sim_params, 'acceleration_limits')
        control_limits = active_data.sim_params.acceleration_limits;
    else
        control_limits = [1]; % Default
        warning('Could not find acceleration limits; using default value: 1');
    end
    
    % Set velocities to 1 for compatibility with the rest of the code
    velocities = [1];
    
    % Check dimensions
    if length(g.N) ~= 2
        warning('Double Integrator should have 2 dimensions (position, velocity)');
    end
    
    fprintf('Model detected: Double Integrator\n');
    
elseif strcmp(model_type, 'dubinsCar')
    % Extract Dubins Car parameters
    if isfield(active_data, 'turning_limits')
        control_limits = active_data.turning_limits;
    elseif isfield(active_data, 'sim_params') && isfield(active_data.sim_params, 'turning_limits')
        control_limits = active_data.sim_params.turning_limits;
    else
        control_limits = [pi/4]; % Default
        warning('Could not find turning limits; using default value: pi/4');
    end
    
    % Get speed
    if isfield(active_data, 'speed')
        speed = active_data.speed;
    elseif isfield(active_data, 'sim_params') && isfield(active_data.sim_params, 'speed')
        speed = active_data.sim_params.speed;
    else
        speed = 5; % Default
        warning('Could not find speed parameter; using default value: 5');
    end
    
    % Set velocities to speed for compatibility with the rest of the code
    velocities = [speed];
    
    % Check dimensions
    if length(g.N) ~= 3
        warning('Dubins Car should have 3 dimensions (x, y, theta)');
    end
    
    fprintf('Model detected: Dubins Car with speed %.2f m/s\n', speed);
else
    error('Unsupported model type: %s', model_type);
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

% Extract reachable set data for selected velocity and control limit
data_value_function = active_data.all_data{velocity_idx, control_idx};

% Extract full time data if available
if isfield(active_data, 'all_data_full')
    data_value_function_full = active_data.all_data_full{velocity_idx, control_idx};
else
    warning('Full-time data not available. Some optimized methods may not work correctly.');
    data_value_function_full = data_value_function;  % Use single time slice as fallback
end

% Load complementary data (FRS for BRS trajectory, BRS for FRS trajectory)
data_complement = [];
if strcmp(trajectory_type, 'brs') && use_frs_constraint && ~isempty(frs_folder)
    % Load FRS data for BRS trajectory with constraints
    frs_combined_file = fullfile(frs_folder, 'frs_combined_results.mat');
    if ~exist(frs_combined_file, 'file')
        % Try alternate filename
        frs_combined_file = fullfile(frs_folder, 'brs_combined_results.mat');
    end
    
    if exist(frs_combined_file, 'file')
        fprintf('Loading FRS data for safety constraints from: %s\n', frs_combined_file);
        frs_data = load(frs_combined_file);
        
        % Find matching velocity and control limit in FRS data
        if strcmp(model_type, 'bicycle')
            % For bicycle models
            v_idx = find(frs_data.velocities == velocities(velocity_idx), 1);
            if isempty(v_idx)
                % Find closest
                [~, v_idx] = min(abs(frs_data.velocities - velocities(velocity_idx)));
            end
            
            if isfield(frs_data, 'dvmax_values')
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
        else
            % For other models, just use the first control limit
            v_idx = 1;
            c_idx = 1;
        end
        
        data_complement = frs_data.all_data{v_idx, c_idx};
    else
        warning('FRS results file not found. FRS constraints will be disabled.');
        use_frs_constraint = false;
    end
elseif strcmp(trajectory_type, 'frs') && ~isempty(brs_folder)
    % For FRS trajectories, we may want to use the BRS data for visualization purposes
    % This is optional but helps with comparing the two types of trajectories
    brs_combined_file = fullfile(brs_folder, 'brs_combined_results.mat');
    
    if exist(brs_combined_file, 'file')
        fprintf('Loading BRS data for comparison from: %s\n', brs_combined_file);
        brs_data = load(brs_combined_file);
        
        % Find matching velocity and control limit
        if strcmp(model_type, 'bicycle')
            v_idx = find(brs_data.velocities == velocities(velocity_idx), 1);
            if isempty(v_idx)
                [~, v_idx] = min(abs(brs_data.velocities - velocities(velocity_idx)));
            end
            
            if isfield(brs_data, 'dvmax_values')
                c_idx = find(brs_data.dvmax_values == control_limits(control_idx), 1);
                if isempty(c_idx)
                    [~, c_idx] = min(abs(brs_data.dvmax_values - control_limits(control_idx)));
                end
            else
                c_idx = find(brs_data.mzmax_values == control_limits(control_idx), 1);
                if isempty(c_idx)
                    [~, c_idx] = min(abs(brs_data.mzmax_values - control_limits(control_idx)));
                end
            end
        else
            % For other models, just use the first control limit
            v_idx = 1;
            c_idx = 1;
        end
        
        data_complement = brs_data.all_data{v_idx, c_idx};
    end
end

%% Compute or load arrival time function
% The arrival time function improves trajectory computation
has_arrival_time = false;
arrival_time = [];

if exist('compute_arrival_time', 'file') && ~isempty(data_value_function_full) && strcmp(trajectory_method, 'arrival')
    fprintf('Computing time-of-arrival function for improved trajectory planning...\n');
    try
        arrival_time = compute_arrival_time(data_value_function_full, tau);
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
    if strcmp(model_type, 'bicycle') && is_steered_model
        % For 3D steered bicycle model
        viz_options.deltaSlices = [-0.1, 0, 0.1]; % Default slices
    elseif strcmp(model_type, 'dubinsCar')
        % For Dubins Car, show slices at different headings
        viz_options.sliceDim = 3; % Slice along heading dimension
        viz_options.slices = [0, pi/4, -pi/4]; % Different heading angles
    end
    
    try
        % Check if unified visualization function exists
        if exist('visualize_reachability_results', 'file')
            visualize_reachability_results(active_folder, viz_options);
        else
            warning('visualize_reachability_results function not found. Skipping reachable set visualization.');
            
            % Provide simple fallback visualization
            figure('Name', 'Reachable Set Visualization');
            
            if strcmp(model_type, 'bicycle')
                if is_steered_model
                    % Show a slice for steered bicycle (gamma-beta plane at delta=0)
                    delta_values = g.xs{3}(1, 1, :);
                    center_idx = ceil(length(delta_values)/2);
                    data_slice = squeeze(data_value_function(:,:,center_idx));
                    target_slice = squeeze(data0(:,:,center_idx));
                    
                    contour(g.xs{2}(:,:,1)*180/pi, g.xs{1}(:,:,1)*180/pi, data_slice, [0 0], 'LineWidth', 2);
                    hold on;
                    contour(g.xs{2}(:,:,1)*180/pi, g.xs{1}(:,:,1)*180/pi, target_slice, [0 0], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'g');
                    xlabel('Sideslip Angle (degrees)');
                    ylabel('Yaw Rate (degrees/s)');
                    title(sprintf('BRS/FRS at Steering Angle = %.1f°', delta_values(center_idx)*180/pi));
                else
                    % For standard bicycle
                    contour(g.xs{2}*180/pi, g.xs{1}*180/pi, data_value_function, [0 0], 'LineWidth', 2);
                    hold on;
                    contour(g.xs{2}*180/pi, g.xs{1}*180/pi, data0, [0 0], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'g');
                    xlabel('Sideslip Angle (degrees)');
                    ylabel('Yaw Rate (degrees/s)');
                    title('BRS/FRS Boundary');
                end
            elseif strcmp(model_type, 'doubleInt')
                % For Double Integrator
                contour(g.xs{1}, g.xs{2}, data_value_function, [0 0], 'LineWidth', 2);
                hold on;
                contour(g.xs{1}, g.xs{2}, data0, [0 0], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'g');
                xlabel('Position');
                ylabel('Velocity');
                title('Double Integrator BRS/FRS Boundary');
            elseif strcmp(model_type, 'dubinsCar')
                % For Dubins Car, show slice at zero heading
                theta_values = g.xs{3}(1, 1, :);
                center_idx = ceil(length(theta_values)/2);
                data_slice = squeeze(data_value_function(:,:,center_idx));
                target_slice = squeeze(data0(:,:,center_idx));
                
                contour(g.xs{1}(:,:,1), g.xs{2}(:,:,1), data_slice, [0 0], 'LineWidth', 2);
                hold on;
                contour(g.xs{1}(:,:,1), g.xs{2}(:,:,1), target_slice, [0 0], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'g');
                xlabel('X Position');
                ylabel('Y Position');
                title(sprintf('BRS/FRS at Heading = %.1f°', theta_values(center_idx)*180/pi));
            end
            
            grid on;
            legend('Reachable Set', 'Target Set');
        end
    catch err
        warning('Error during reachable set visualization: %s', err.message);
    end
end

%% Compute or load trajectory
if visualize_trajectory
    % Validate initial state dimension
    if strcmp(model_type, 'bicycle') && is_steered_model && length(xinit) ~= 3
        error('Initial state must have 3 dimensions [gamma; beta; delta] for steered bicycle model');
    elseif strcmp(model_type, 'bicycle') && ~is_steered_model && length(xinit) ~= 2
        error('Initial state must have 2 dimensions [gamma; beta] for standard bicycle model');
    elseif strcmp(model_type, 'doubleInt') && length(xinit) ~= 2
        error('Initial state must have 2 dimensions [position; velocity] for Double Integrator model');
    elseif strcmp(model_type, 'dubinsCar') && length(xinit) ~= 3
        error('Initial state must have 3 dimensions [x; y; theta] for Dubins Car model');
    end
    
    % Set up dynamic system for trajectory computation
    if strcmp(model_type, 'bicycle')
        % Update vehicle parameters with selected velocity and control limits
        params = base_params;
        params(2) = velocities(velocity_idx);  % Set velocity
        
        if is_steered_model
            % For steered bicycle model
            params(7) = control_limits(control_idx);  % Set dv_max
            fprintf('Using steered bicycle model with Vx = %d m/s, dv_max = %.2f rad/s (%.1f deg/s)\n', ...
                velocities(velocity_idx), control_limits(control_idx), control_limits(control_idx)*180/pi);
            
            % Create the dynamic system object
            dynSys = NonlinearBicycleSteered(xinit, params);
        else
            % For standard bicycle model
            params(7) = control_limits(control_idx);   % Set Mzmax
            params(8) = -control_limits(control_idx);  % Set Mzmin
            fprintf('Using standard bicycle model with Vx = %d m/s, Mzmax = %d N·m\n', ...
                velocities(velocity_idx), control_limits(control_idx));
            
            % Create the dynamic system object
            dynSys = NonlinearBicycle(xinit, params);
        end
    elseif strcmp(model_type, 'doubleInt')
        % For Double Integrator
        acc_limit = control_limits(control_idx);
        urange = [-acc_limit, acc_limit];
        
        % Get dimensions and disturbance if available
        if isfield(active_data, 'dimensions')
            dims = active_data.dimensions;
        elseif isfield(active_data, 'sim_params') && isfield(active_data.sim_params, 'dimensions')
            dims = active_data.sim_params.dimensions;
        else
            dims = 1:2;  % Default dimensions
        end
        
        if isfield(active_data, 'disturbance_range')
            drange = active_data.disturbance_range;
        elseif isfield(active_data, 'sim_params') && isfield(active_data.sim_params, 'disturbance_range')
            drange = active_data.sim_params.disturbance_range;
        else
            drange = [0, 0];  % Default - no disturbance
        end
        
        fprintf('Using Double Integrator model with acceleration limits = [%.2f, %.2f]\n', -acc_limit, acc_limit);
        
        % Create the Double Integrator object
        dynSys = DoubleInt(xinit, urange, drange, dims);
    elseif strcmp(model_type, 'dubinsCar')
        % For Dubins Car
        % Get turning limit
        turn_limit = control_limits(control_idx);
        wRange = [-turn_limit, turn_limit];
        
        % Get speed
        if isfield(active_data, 'speed')
            speed = active_data.speed;
        elseif isfield(active_data, 'sim_params') && isfield(active_data.sim_params, 'speed')
            speed = active_data.sim_params.speed;
        else
            speed = velocities(velocity_idx);  % Use stored velocity as speed
        end
        
        % Get dimensions and disturbance if available
        if isfield(active_data, 'dimensions')
            dims = active_data.dimensions;
        elseif isfield(active_data, 'sim_params') && isfield(active_data.sim_params, 'dimensions')
            dims = active_data.sim_params.dimensions;
        else
            dims = 1:3;  % Default dimensions
        end
        
        if isfield(active_data, 'disturbance_range')
            drange = active_data.disturbance_range;
        elseif isfield(active_data, 'sim_params') && isfield(active_data.sim_params, 'disturbance_range')
            drange = active_data.sim_params.disturbance_range;
        else
            drange = {[0;0;0], [0;0;0]};  % Default - no disturbance
        end
        
        fprintf('Using Dubins Car model with speed = %.2f m/s, turning limits = [%.2f, %.2f] rad/s\n', ...
            speed, -turn_limit, turn_limit);
        
        % Create the Dubins Car object
        dynSys = DubinsCar(xinit, wRange, speed, drange, dims);
    end
    
    % If this is an FRS trajectory, validate that initial state is in/near target set
    if strcmp(trajectory_type, 'frs')
        % Check if initial state is in target set (for FRS trajectories)
        target_value = eval_u(g, data0, xinit);
        if target_value > 0
            warning('For FRS trajectories, initial state should be in the target set.');
            fprintf('Current initial state has target set value: %.4f\n', target_value);
        end
    end
    
    % Set control mode based on trajectory type
    if strcmp(trajectory_type, 'brs')
        uMode = 'min';  % For BRS trajectories
        fprintf('Computing BRS trajectory (control minimizes value function to reach target)\n');
    else % 'frs'
        uMode = 'max';  % For FRS trajectories
        fprintf('Computing FRS trajectory (control maximizes value function to escape target)\n');
    end
    
    % Compute or load trajectory
    if compute_new_trajectory
        fprintf('Computing trajectory for initial state: ');
        if strcmp(model_type, 'bicycle') && is_steered_model
            fprintf('[%.1f°/s, %.1f°, %.1f°]...\n', ...
                xinit(1)*180/pi, xinit(2)*180/pi, xinit(3)*180/pi);
        elseif strcmp(model_type, 'bicycle')
            fprintf('[%.1f°/s, %.1f°]...\n', ...
                xinit(1)*180/pi, xinit(2)*180/pi);
        elseif strcmp(model_type, 'doubleInt')
            fprintf('[%.2f, %.2f]...\n', xinit(1), xinit(2));
        elseif strcmp(model_type, 'dubinsCar')
            fprintf('[%.2f, %.2f, %.1f°]...\n', ...
                xinit(1), xinit(2), xinit(3)*180/pi);
        end
        
        % Check if initial state is valid for the selected trajectory type
        if strcmp(trajectory_type, 'brs')
            % For BRS trajectory, check if initial state is in BRS
            initial_value = eval_u(g, data_value_function, xinit);
            if initial_value > 0
                error('Initial state is not in the BRS (value = %.4f). Try a different initial state.', initial_value);
            end
        else % 'frs'
            % For FRS trajectory, no strict requirement, but ideally close to target
            initial_value = eval_u(g, data_value_function, xinit);
            if initial_value > 0
                warning('Initial state is not in the FRS (value = %.4f). Results may be unexpected.', initial_value);
            end
        end
        
        % Select trajectory computation method
        if strcmp(trajectory_method, 'legacy')
            % Legacy trajectory computation methods depend on model type
            fprintf('Using legacy %s method...\n', trajectory_method);
            
            % Legacy methods may not fully support FRS trajectories or all model types
            if strcmp(trajectory_type, 'frs')
                warning('Legacy methods may not fully support FRS trajectories. Consider using optimized methods.');
            end
            
            % Each model type needs specific legacy computation handling
            if strcmp(model_type, 'bicycle') && is_steered_model
                % Steered bicycle model
                try
                    legacy_opts = struct();
                    legacy_opts.velocityIdx = velocity_idx;
                    legacy_opts.dvMaxIdx = control_idx;
                    legacy_opts.visualize = false;
                    legacy_opts.maxTime = max_time;
                    legacy_opts.uMode = uMode;
                    
                    if strcmp(trajectory_type, 'brs') && use_frs_constraint
                        legacy_opts.useFRS = true;
                        legacy_opts.frs_folder = frs_folder;
                        legacy_opts.frsWeight = frs_weight;
                    else
                        legacy_opts.useFRS = false;
                    end
                    
                    [traj, traj_tau, traj_u, metrics] = compute_trajectory_steered_from_folders(active_folder, xinit, legacy_opts);
                catch err
                    error('Error in legacy trajectory computation: %s', err.message);
                end
            elseif strcmp(model_type, 'bicycle')
                % Standard bicycle model
                try
                    legacy_opts = struct();
                    legacy_opts.velocityIdx = velocity_idx;
                    legacy_opts.mzMaxIdx = control_idx;
                    legacy_opts.visualize = false;
                    legacy_opts.maxTime = max_time;
                    legacy_opts.uMode = uMode;
                    
                    if strcmp(trajectory_type, 'brs') && use_frs_constraint
                        legacy_opts.useFRS = true;
                        legacy_opts.frs_folder = frs_folder;
                        legacy_opts.frsWeight = frs_weight;
                    else
                        legacy_opts.useFRS = false;
                    end
                    
                    [traj, traj_tau, traj_u, metrics] = compute_trajectory_from_folders(active_folder, xinit, legacy_opts);
                catch err
                    error('Error in legacy trajectory computation: %s', err.message);
                end
            else
                % Legacy methods don't explicitly support other models
                warning('Legacy trajectory computation methods may not support %s model. Using computeOptTraj instead.', model_type);
                
                try
                    % Use bare computeOptTraj function
                    TrajextraArgs = struct();
                    TrajextraArgs.uMode = uMode;
                    TrajextraArgs.visualize = false;
                    
                    % Flip data time points for backward direction
                    dataTraj = flip(data_value_function_full, length(g.N) + 1);
                    tau_reversed = flip(tau);
                    
                    [traj, traj_tau, traj_u] = computeOptTraj(g, dataTraj, tau_reversed, dynSys, TrajextraArgs);
                    
                    % Create a basic metrics structure
                    metrics = struct();
                    metrics.time_to_target = traj_tau(end) - traj_tau(1);
                    metrics.final_set_value = eval_u(g, data0, traj(:,end));
                    metrics.control_energy = sum(sum(traj_u.^2));
                    
                    if strcmp(trajectory_type, 'brs')
                        metrics.reached_target = (metrics.final_set_value <= 0);
                    else
                        metrics.escaped_target = (metrics.final_set_value > 0);
                    end
                catch err
                    error('Error in computeOptTraj: %s', err.message);
                end
            end
        else
            % Use the optimized trajectory computation
            fprintf('Using optimized trajectory computation with method: %s\n', trajectory_method);
            
            % Check if computeOptimizedTrajectory function exists
            if ~exist('computeOptimizedTrajectory', 'file')
                error('computeOptimizedTrajectory function not found. Please add it to your MATLAB path.');
            end
            
            % Setup options for optimized computation
            opt_options = struct();
            opt_options.method = trajectory_method;
            opt_options.uMode = uMode;
            opt_options.maxTime = max_time;
            opt_options.dt = (tau(2) - tau(1))/5;  % Use smaller time step for better integration
            opt_options.visualize = false;
            opt_options.finalSet = data0;  % Use target set from data
            
            % Add arrival time if available
            if has_arrival_time
                opt_options.arrivalTime = arrival_time;
            end
            
            % Add FRS constraints if enabled for BRS trajectory
            if strcmp(trajectory_type, 'brs') && use_frs_constraint && ~isempty(data_complement)
                opt_options.useFRS = true;
                opt_options.data_frs = data_complement;
                opt_options.frsWeight = frs_weight;
            end
            
            % Handle time direction differently for FRS trajectories
            if strcmp(trajectory_type, 'frs')
                opt_options.reverseFRS = true;  % Flag indicating this is an FRS trajectory
            end
            
            try
                % Compute trajectory using optimized method
                [traj, traj_tau, traj_u, metrics] = computeOptimizedTrajectory(g, data_value_function_full, tau, xinit, dynSys, opt_options);
            catch err
                error('Error in optimized trajectory computation: %s', err.message);
            end
        end
        
        % Save computed trajectory
        save(trajectory_file, 'traj', 'traj_tau', 'traj_u', 'metrics', 'xinit', 'active_folder', ...
            'trajectory_method', 'trajectory_type', 'params', 'velocity_idx', 'control_idx', 'model_type');
        fprintf('Trajectory computed and saved to %s\n', trajectory_file);
    else
        % Load a previously computed trajectory
        if exist(trajectory_file, 'file')
            fprintf('Loading trajectory from %s\n', trajectory_file);
            load(trajectory_file, 'traj', 'traj_tau', 'traj_u', 'metrics', 'xinit', 'trajectory_type', 'model_type');
            
            % Check if trajectory file contains the required data
            if ~exist('traj', 'var') || ~exist('traj_tau', 'var')
                error('Trajectory file does not contain required data');
            end
            
            % If model_type isn't in the saved file, try to infer it
            if ~exist('model_type', 'var')
                % Try to infer model type from trajectory dimensions
                if size(traj, 1) == 3 && ~isempty(strfind(trajectory_file, 'dubins'))
                    model_type = 'dubinsCar';
                elseif size(traj, 1) == 2 && ~isempty(strfind(trajectory_file, 'double'))
                    model_type = 'doubleInt';
                else
                    % Default to bicycle model
                    model_type = 'bicycle';
                    is_steered_model = (size(traj, 1) == 3);
                end
            end
        else
            error('Trajectory file not found: %s', trajectory_file);
        end
    end
    
    %% Display trajectory metrics
    fprintf('\n=== %s Trajectory Metrics ===\n', upper(trajectory_type));
    fprintf('Computation Method: %s\n', trajectory_method);
    
    if isfield(metrics, 'time_to_target')
        if strcmp(trajectory_type, 'brs')
            fprintf('Time to target: %.2f seconds\n', metrics.time_to_target);
        else % FRS
            fprintf('Escape time: %.2f seconds\n', metrics.time_to_target);
        end
    end
    
    if isfield(metrics, 'final_set_value')
        if strcmp(trajectory_type, 'brs')
            if metrics.final_set_value <= 0
                fprintf('Target reached! (final value: %.4f)\n', metrics.final_set_value);
            else
                fprintf('Target NOT reached (final value: %.4f)\n', metrics.final_set_value);
            end
        else % FRS
            fprintf('Final value: %.4f\n', metrics.final_set_value);
            if metrics.final_set_value <= 0
                fprintf('Warning: FRS trajectory ended inside target set\n');
            else
                fprintf('Successfully escaped target set\n');
            end
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
    fprintf('\nVisualizing %s trajectory...\n', upper(trajectory_type));
    
    try
        % Create figure title based on model type
        if strcmp(model_type, 'bicycle')
            figure_title = sprintf('%s Trajectory in State Space (%s Model)', upper(trajectory_type));
        elseif strcmp(model_type, 'doubleInt')
            figure_title = sprintf('%s Trajectory in State Space (Double Integrator)', upper(trajectory_type));
        elseif strcmp(model_type, 'dubinsCar')
            figure_title = sprintf('%s Trajectory in State Space (Dubins Car)', upper(trajectory_type));
        else
            figure_title = sprintf('%s Trajectory in State Space', upper(trajectory_type));
        end
        
        figure('Name', figure_title);
        
        % Visualization based on model type
        if strcmp(model_type, 'bicycle')
            if is_steered_model
                % For 3D model, show slices at the steering angle
                delta_values = g.xs{3}(1, 1, :);
                center_idx = ceil(length(delta_values)/2);
                
                % Find the index closest to the initial steering angle
                [~, delta_idx] = min(abs(delta_values - xinit(3)));
                
                % Extract 2D slices
                data_slice = squeeze(data_value_function(:,:,delta_idx));
                target_slice = squeeze(data0(:,:,center_idx));
                
                % Plot the slice
                xs1_deg = g.xs{1}(:,:,1) * 180/pi;  % yaw rate
                xs2_deg = g.xs{2}(:,:,1) * 180/pi;  % sideslip angle
                
                hold on;
                [~, h1] = contour(xs2_deg, xs1_deg, data_slice, [0, 0], 'LineWidth', 2, 'Color', 'k');
                [~, h2] = contour(xs2_deg, xs1_deg, target_slice, [0, 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                
                % Also plot BRS data if this is an FRS trajectory and we have BRS data
                if strcmp(trajectory_type, 'frs') && ~isempty(data_complement)
                    brs_slice = squeeze(data_complement(:,:,delta_idx));
                    [~, h_brs] = contour(xs2_deg, xs1_deg, brs_slice, [0, 0], 'LineWidth', 2, 'Color', 'b', 'LineStyle', ':');
                end
                
                % Convert trajectory to degrees for plotting
                traj_deg = traj * 180/pi;
                
                % Plot trajectory in state space (sideslip vs yaw rate)
                h3 = plot(traj_deg(2,:), traj_deg(1,:), 'b-', 'LineWidth', 2);
                h4 = plot(traj_deg(2,1), traj_deg(1,1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
                
                % For BRS trajectories, final point is "target reached"
                % For FRS trajectories, final point is "escaped from target"
                if strcmp(trajectory_type, 'brs')
                    h5 = plot(traj_deg(2,end), traj_deg(1,end), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
                else
                    h5 = plot(traj_deg(2,end), traj_deg(1,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
                end
                
                xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
                ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
                
                if strcmp(trajectory_type, 'brs')
                    main_set_name = 'BRS';
                    title(sprintf('BRS Trajectory in State Space (δ = %.1f°)', delta_values(delta_idx)*180/pi), 'FontSize', 14);
                    if ~isempty(data_complement)
                        legend([h1, h2, h3, h4, h5], 'BRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State (Target Reached)', 'Location', 'best');
                    else
                        legend([h1, h2, h3, h4, h5], 'BRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State (Target Reached)', 'Location', 'best');
                    end
                else % FRS
                    main_set_name = 'FRS';
                    title(sprintf('FRS Trajectory in State Space (δ = %.1f°)', delta_values(delta_idx)*180/pi), 'FontSize', 14);
                    if ~isempty(data_complement)
                        legend([h1, h2, h_brs, h3, h4, h5], 'FRS Boundary', 'Target Set', 'BRS Boundary', 'Trajectory', 'Initial State', 'Final State (Escaped)', 'Location', 'best');
                    else
                        legend([h1, h2, h3, h4, h5], 'FRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State (Escaped)', 'Location', 'best');
                    end
                end
                
                grid on;
                
                % Also create a figure showing the steering angle over time
                figure('Name', 'Steering Angle vs Time');
                plot(traj_tau, traj_deg(3,:), 'r-', 'LineWidth', 2);
                xlabel('Time (s)', 'FontSize', 12);
                ylabel('Steering Angle (degrees)', 'FontSize', 12);
                title(sprintf('%s Trajectory: Steering Angle vs Time', main_set_name), 'FontSize', 14);
                grid on;
                
            else
                % For 2D model
                hold on;
                [~, h1] = contour(g.xs{2}*180/pi, g.xs{1}*180/pi, data_value_function, [0, 0], 'LineWidth', 2, 'Color', 'k');
                [~, h2] = contour(g.xs{2}*180/pi, g.xs{1}*180/pi, data0, [0, 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                
                % Also plot BRS data if this is an FRS trajectory and we have BRS data
                if strcmp(trajectory_type, 'frs') && ~isempty(data_complement)
                    [~, h_brs] = contour(g.xs{2}*180/pi, g.xs{1}*180/pi, data_complement, [0, 0], 'LineWidth', 2, 'Color', 'b', 'LineStyle', ':');
                end
                
                % Convert trajectory to degrees for plotting
                traj_deg = traj * 180/pi;
                
                % Plot trajectory
                h3 = plot(traj_deg(2,:), traj_deg(1,:), 'b-', 'LineWidth', 2);
                h4 = plot(traj_deg(2,1), traj_deg(1,1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
                
                % For BRS trajectories, final point is "target reached"
                % For FRS trajectories, final point is "escaped from target"
                if strcmp(trajectory_type, 'brs')
                    h5 = plot(traj_deg(2,end), traj_deg(1,end), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
                else
                    h5 = plot(traj_deg(2,end), traj_deg(1,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
                end
                
                xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
                ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
                
                if strcmp(trajectory_type, 'brs')
                    main_set_name = 'BRS';
                    title('BRS Trajectory in State Space', 'FontSize', 14);
                    if ~isempty(data_complement)
                        legend([h1, h2, h3, h4, h5], 'BRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State (Target Reached)', 'Location', 'best');
                    else
                        legend([h1, h2, h3, h4, h5], 'BRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State (Target Reached)', 'Location', 'best');
                    end
                else % FRS
                    main_set_name = 'FRS';
                    title('FRS Trajectory in State Space', 'FontSize', 14);
                    if ~isempty(data_complement)
                        legend([h1, h2, h_brs, h3, h4, h5], 'FRS Boundary', 'Target Set', 'BRS Boundary', 'Trajectory', 'Initial State', 'Final State (Escaped)', 'Location', 'best');
                    else
                        legend([h1, h2, h3, h4, h5], 'FRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State (Escaped)', 'Location', 'best');
                    end
                end
                
                grid on;
            end
        elseif strcmp(model_type, 'doubleInt')
            % Double Integrator visualization
            hold on;
            [~, h1] = contour(g.xs{1}, g.xs{2}, data_value_function, [0, 0], 'LineWidth', 2, 'Color', 'k');
            [~, h2] = contour(g.xs{1}, g.xs{2}, data0, [0, 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
            
            % Plot trajectory
            h3 = plot(traj(1,:), traj(2,:), 'b-', 'LineWidth', 2);
            h4 = plot(traj(1,1), traj(2,1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
            
            % For BRS trajectories, final point is "target reached"
            % For FRS trajectories, final point is "escaped from target"
            if strcmp(trajectory_type, 'brs')
                h5 = plot(traj(1,end), traj(2,end), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
            else
                h5 = plot(traj(1,end), traj(2,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            end
            
            xlabel('Position', 'FontSize', 12);
            ylabel('Velocity', 'FontSize', 12);
            
            if strcmp(trajectory_type, 'brs')
                main_set_name = 'BRS';
                title('Double Integrator: BRS Trajectory', 'FontSize', 14);
                legend([h1, h2, h3, h4, h5], 'BRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State (Target Reached)', 'Location', 'best');
            else % FRS
                main_set_name = 'FRS';
                title('Double Integrator: FRS Trajectory', 'FontSize', 14);
                legend([h1, h2, h3, h4, h5], 'FRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State (Escaped)', 'Location', 'best');
            end
            
            grid on;
            
        elseif strcmp(model_type, 'dubinsCar')
            % Dubins Car visualization
            if length(g.N) == 3
                % 3D state space, show 2D projections
                
                % Find the theta slice closest to initial heading
                theta_values = g.xs{3}(1, 1, :);
                [~, theta_idx] = min(abs(theta_values - xinit(3)));
                
                % Extract 2D slice at this heading
                data_slice = squeeze(data_value_function(:,:,theta_idx));
                target_slice = squeeze(data0(:,:,theta_idx));
                
                % Plot the slice
                hold on;
                [~, h1] = contour(g.xs{1}(:,:,1), g.xs{2}(:,:,1), data_slice, [0, 0], 'LineWidth', 2, 'Color', 'k');
                [~, h2] = contour(g.xs{1}(:,:,1), g.xs{2}(:,:,1), target_slice, [0, 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                
                % Plot trajectory (x-y projection)
                h3 = plot(traj(1,:), traj(2,:), 'b-', 'LineWidth', 2);
                h4 = plot(traj(1,1), traj(2,1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
                
                if strcmp(trajectory_type, 'brs')
                    h5 = plot(traj(1,end), traj(2,end), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
                else
                    h5 = plot(traj(1,end), traj(2,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
                end
                
                xlabel('X Position', 'FontSize', 12);
                ylabel('Y Position', 'FontSize', 12);
                
                if strcmp(trajectory_type, 'brs')
                    main_set_name = 'BRS';
                    title(sprintf('Dubins Car: BRS Trajectory (θ = %.1f°)', theta_values(theta_idx)*180/pi), 'FontSize', 14);
                    legend([h1, h2, h3, h4, h5], 'BRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State (Target Reached)', 'Location', 'best');
                else % FRS
                    main_set_name = 'FRS';
                    title(sprintf('Dubins Car: FRS Trajectory (θ = %.1f°)', theta_values(theta_idx)*180/pi), 'FontSize', 14);
                    legend([h1, h2, h3, h4, h5], 'FRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State (Escaped)', 'Location', 'best');
                end
                
                grid on;
                
                % Also create a figure showing the heading angle over time
                figure('Name', 'Heading Angle vs Time');
                plot(traj_tau, traj(3,:)*180/pi, 'r-', 'LineWidth', 2);
                xlabel('Time (s)', 'FontSize', 12);
                ylabel('Heading Angle (degrees)', 'FontSize', 12);
                title(sprintf('Dubins Car: %s Trajectory - Heading vs Time', main_set_name), 'FontSize', 14);
                grid on;
            else
                % 2D state space (simplified model)
                hold on;
                [~, h1] = contour(g.xs{1}, g.xs{2}, data_value_function, [0, 0], 'LineWidth', 2, 'Color', 'k');
                [~, h2] = contour(g.xs{1}, g.xs{2}, data0, [0, 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                
                % Plot trajectory
                h3 = plot(traj(1,:), traj(2,:), 'b-', 'LineWidth', 2);
                h4 = plot(traj(1,1), traj(2,1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
                
                if strcmp(trajectory_type, 'brs')
                    h5 = plot(traj(1,end), traj(2,end), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
                else
                    h5 = plot(traj(1,end), traj(2,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
                end
                
                xlabel('X Position', 'FontSize', 12);
                ylabel('Y Position', 'FontSize', 12);
                
                if strcmp(trajectory_type, 'brs')
                    main_set_name = 'BRS';
                    title('Dubins Car: BRS Trajectory', 'FontSize', 14);
                    legend([h1, h2, h3, h4, h5], 'BRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State (Target Reached)', 'Location', 'best');
                else % FRS
                    main_set_name = 'FRS';
                    title('Dubins Car: FRS Trajectory', 'FontSize', 14);
                    legend([h1, h2, h3, h4, h5], 'FRS Boundary', 'Target Set', 'Trajectory', 'Initial State', 'Final State (Escaped)', 'Location', 'best');
                end
                
                grid on;
            end
        end

        % Also visualize control inputs
        figure('Name', 'Control Inputs');
        plot(traj_tau(1:end-1), traj_u, 'r-', 'LineWidth', 2);
        xlabel('Time (s)', 'FontSize', 12);
        
        if strcmp(model_type, 'bicycle')
            if is_steered_model
                ylabel('Steering Rate (rad/s)', 'FontSize', 12);
                title(sprintf('%s Trajectory: Steering Rate Control Input', main_set_name), 'FontSize', 14);
            else
                ylabel('Yaw Moment (N·m)', 'FontSize', 12);
                title(sprintf('%s Trajectory: Yaw Moment Control Input', main_set_name), 'FontSize', 14);
            end
        elseif strcmp(model_type, 'doubleInt')
            ylabel('Acceleration (m/s²)', 'FontSize', 12);
            title(sprintf('Double Integrator: %s Trajectory - Acceleration Control', main_set_name), 'FontSize', 14);
        elseif strcmp(model_type, 'dubinsCar')
            ylabel('Turning Rate (rad/s)', 'FontSize', 12);
            title(sprintf('Dubins Car: %s Trajectory - Turning Rate Control', main_set_name), 'FontSize', 14);
        end
        grid on;
        
        % Visualize physical trajectory if possible
        if exist('visualizeCarTrajectory', 'file')
            fprintf('Visualizing physical trajectory...\n');
            
            if strcmp(model_type, 'bicycle') || strcmp(model_type, 'dubinsCar')
                % Get the velocity for this simulation
                if strcmp(model_type, 'bicycle')
                    vx = velocities(velocity_idx);
                else
                    % For Dubins Car, use the speed parameter
                    if isfield(active_data, 'speed')
                        vx = active_data.speed;
                    elseif isfield(active_data, 'sim_params') && isfield(active_data.sim_params, 'speed')
                        vx = active_data.sim_params.speed;
                    else
                        vx = velocities(velocity_idx);  % Fallback to velocity
                    end
                end
                
                try
                    % Choose border color based on trajectory type
                    if strcmp(trajectory_type, 'brs')
                        % Use blue scheme for BRS
                        border_color = [0, 0, 0.8];  % Dark blue
                    else
                        % Use red scheme for FRS
                        border_color = [0.8, 0, 0];  % Dark red
                    end
                    
                    visualizeCarTrajectory(traj, traj_tau, g, data_value_function, data0, vx, ...
                        'x0', 0, 'y0', 0, 'psi0', 0, ...
                        'saveVideo', save_video, ...
                        'videoFile', video_file, ...
                        'carLength', car_length, ...
                        'carWidth', car_width, ...
                        'wheelBase', wheel_base, ...
                        'gridSize', grid_size, ...
                        'playSpeed', 1.0, ...
                        'isBRS', strcmp(trajectory_type, 'brs')); % Add flag indicating if BRS or FRS
                    
                    if save_video
                        fprintf('Car trajectory visualization saved to: %s\n', video_file);
                    end
                catch err
                    warning('Error in car trajectory visualization: %s', err.message);
                    
                    % Try again without video if the error is video-related
                    if save_video && (contains(err.message, 'VideoWriter') || contains(err.message, 'video'))
                        fprintf('Attempting visualization without video recording...\n');
                        try
                            visualizeCarTrajectory(traj, traj_tau, g, data_value_function, data0, vx, ...
                                'x0', 0, 'y0', 0, 'psi0', 0, ...
                                'saveVideo', false, ...
                                'carLength', car_length, ...
                                'carWidth', car_width, ...
                                'wheelBase', wheel_base, ...
                                'gridSize', grid_size, ...
                                'playSpeed', 1.0, ...
                                'isBRS', strcmp(trajectory_type, 'brs'));
                            fprintf('Car trajectory visualization completed without video recording.\n');
                        catch inner_err
                            warning('Visualization failed: %s', inner_err.message);
                        end
                    end
                end
            elseif strcmp(model_type, 'doubleInt')
                % For Double Integrator, we could implement a simple particle visualization
                % But this would need a custom visualization function
                fprintf('Physical visualization for Double Integrator not implemented yet.\n');
                
                % Simple alternative: plot position vs time and phase portrait
                figure('Name', 'Double Integrator Position vs Time');
                subplot(2,1,1);
                plot(traj_tau, traj(1,:), 'b-', 'LineWidth', 2);
                xlabel('Time (s)', 'FontSize', 12);
                ylabel('Position', 'FontSize', 12);
                title('Double Integrator: Position vs Time', 'FontSize', 14);
                grid on;
                
                subplot(2,1,2);
                plot(traj_tau, traj(2,:), 'r-', 'LineWidth', 2);
                xlabel('Time (s)', 'FontSize', 12);
                ylabel('Velocity', 'FontSize', 12);
                title('Double Integrator: Velocity vs Time', 'FontSize', 14);
                grid on;
            end
        else
            warning('visualizeCarTrajectory function not found. Skipping physical trajectory visualization.');
        end
        
    catch err
        warning('Error during trajectory visualization: %s', err.message);
    end
end

fprintf('\nReachability visualization tool execution complete.\n');
end