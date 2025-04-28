function compute_reachability_wrapper()
% COMPUTE_REACHABILITY_WRAPPER Script to run reachability computations with user-defined parameters
%
% This wrapper script allows you to easily configure and run reachability computations
% with different parameter combinations using the unified compute_reachable_set function.
%
% To use:
% 1. Modify the configuration parameters in the section below
% 2. Run the script
%
% The script will run the computation and visualize the results as specified.

clear; clc;

%% Configuration Parameters - MODIFY THESE VALUES
% ---------------------------------------------------

%% Select if you want to load or produce results
generate_results = true;     % Set to false to load existing results
visualize_results = true;    % Set to false to skip visualization
save_plots = false;          % Set to true to save visualization figures

%% Computation type
direction = 'backward';      % Options: 'backward' (BRS) or 'forward' (FRS)
controlType = 'dv';          % Options: 'mz' (yaw moment) or 'dv' (steering rate)

%% Vehicle parameters
velocities = [30];           % Vehicle velocities to test [m/s]

%% Control limits
% - If controlType = 'mz': These are max yaw moments [N·m]
% - If controlType = 'dv': These are max steering rates [deg/s] - will be converted to [rad/s]
control_limits = [20];    % For 'mz' control: [10000] means 10,000 N·m
                             % For 'dv' control: [20] means 20 deg/s

%% Time parameters
tMax = 1.0;                  % Maximum simulation time [s]
dt = 0.05;                   % Time step [s]

%% Grid parameters
% Grid size: Number of grid points in each dimension
% - For 'mz': [nx, ny] for [yaw rate, sideslip]
% - For 'dv': [nx, ny, nz] for [yaw rate, sideslip, steering]
gridSize = [101, 101, 51];               % Empty = use defaults based on controlType
                             % Default for 'mz': [71, 71]
                             % Default for 'dv': [51, 51, 51]

% Grid limits in degrees (will be converted to radians)
% - For 'mz': [gamma_min, beta_min] and [gamma_max, beta_max]
% - For 'dv': [gamma_min, beta_min, delta_min] and [gamma_max, beta_max, delta_max]
gridMin_deg = [-150, -25, -10];            % Empty = use defaults based on controlType
gridMax_deg = [150, 25, 10];            % Empty = use defaults based on controlType
                             % Default for 'mz': [-150, -25] to [150, 25]
                             % Default for 'dv': [-150, -25, -10] to [150, 25, 10]

% Target set size in degrees (will be converted to radians)
% - For 'mz': [gamma_size, beta_size]
% - For 'dv': [gamma_size, beta_size, delta_size]
targetSize_deg = [15, 2, 1];         % Empty = use defaults based on controlType
                             % Default for 'mz': [15, 6]
                             % Default for 'dv': [15, 3, 1]

%% Advanced parameters
uMode = 'min';                  % Control strategy: 'min', 'max', or '' (empty = use default)
                             % Default for 'backward': 'min'
                             % Default for 'forward': 'max'

accuracy = 'veryHigh';           % Numerical accuracy: 'low', 'medium', 'high', 'veryhigh'

%% Path to existing results folder (only used if generate_results = false)
existing_result_folder = '';  % Path to the results folder to load

%% Visualization options
% Types of plots to generate (cell array of strings)
% Options for 'mz' control: 'control', 'detailed', 'comparison', 'derivative', 'velocity_stack'
% Options for 'dv' control: 'slices', 'detailed', 'comparison'
plot_types = {'control', 'detailed'};  % Default visualization types

% Whether to show 3D visualization for steered bicycle model (only applies if controlType = 'dv')
show_3d_viz = false;

% ---------------------------------------------------
% END OF CONFIGURATION PARAMETERS - DO NOT EDIT BELOW THIS LINE UNLESS YOU KNOW WHAT YOU'RE DOING
% ---------------------------------------------------

%% Apply parameter conversions and defaults
% Convert steering rate limits from degrees to radians if using steering control
if strcmp(controlType, 'dv')
    control_limits = deg2rad(control_limits);
end

% Apply defaults for grid size if empty
if isempty(gridSize)
    if strcmp(controlType, 'mz')
        gridSize = [71, 71];
    else
        gridSize = [51, 51, 51];
    end
end

% Apply defaults for grid limits if empty
if isempty(gridMin_deg)
    if strcmp(controlType, 'mz')
        gridMin_deg = [-150, -25];
        gridMax_deg = [150, 25];
    else
        gridMin_deg = [-150, -25, -10];
        gridMax_deg = [150, 25, 10];
    end
elseif isempty(gridMax_deg)
    error('If gridMin_deg is specified, gridMax_deg must also be specified');
end

% Convert grid limits from degrees to radians
gridMin = deg2rad(gridMin_deg);
gridMax = deg2rad(gridMax_deg);

% Apply defaults for target set size if empty
if isempty(targetSize_deg)
    if strcmp(controlType, 'mz')
        targetSize_deg = [15, 6];
    else
        targetSize_deg = [15, 3, 1];
    end
end

% Convert target set size from degrees to radians
targetSize = deg2rad(targetSize_deg);

% Apply default control mode based on direction if empty
if isempty(uMode)
    if strcmp(direction, 'backward')
        uMode = 'min';  % For target reaching in BRS
    else
        uMode = 'max';  % For maximizing reachable area in FRS
    end
end

%% Display configuration summary
fprintf('\n=== Reachability Computation Configuration ===\n');
fprintf('Computation: %s (%s direction)\n', upper(direction), direction);
fprintf('Control Type: %s\n', controlType);
fprintf('Velocities: %s m/s\n', mat2str(velocities));

if strcmp(controlType, 'mz')
    fprintf('Yaw Moment Limits: %s N·m\n', mat2str(control_limits));
else
    fprintf('Steering Rate Limits: %s deg/s\n', mat2str(rad2deg(control_limits)));
end

fprintf('Grid Size: %s\n', mat2str(gridSize));
fprintf('Grid Limits: %s to %s degrees\n', mat2str(gridMin_deg), mat2str(gridMax_deg));
fprintf('Target Size: %s degrees\n', mat2str(targetSize_deg));
fprintf('Time Parameters: tMax = %.2f s, dt = %.3f s\n', tMax, dt);
fprintf('Control Strategy: %s\n', uMode);
fprintf('Accuracy Level: %s\n', accuracy);
fprintf('=============================================\n\n');

% Initialize result folder variable
result_folder = '';

%% Run computation or load existing results
if generate_results
    fprintf('Starting %s computation...\n', upper(direction));
    
    % Call the consolidated compute_reachable_set function
    result_folder = compute_reachable_set(velocities, control_limits, ...
        'direction', direction, ...
        'controlType', controlType, ...
        'tMax', tMax, ...
        'dt', dt, ...
        'gridSize', gridSize, ...
        'gridMin', gridMin, ...
        'gridMax', gridMax, ...
        'targetSize', targetSize, ...
        'uMode', uMode, ...
        'accuracy', accuracy, ...
        'visualize', true);
else
    % Load existing results
    if isempty(existing_result_folder) || ~exist(existing_result_folder, 'dir')
        error('For loading existing results, you must specify a valid existing_result_folder');
    end
    
    result_folder = existing_result_folder;
    fprintf('Using existing results from: %s\n', result_folder);
end

%% Visualize results if requested
if visualize_results && ~isempty(result_folder)
    fprintf('Generating visualizations...\n');
    
    % Determine appropriate visualization function based on model type
    if strcmp(controlType, 'mz')
        % For NonlinearBicycle model
        if exist('visualize_reachability_results', 'file')
            % Use unified visualization function if available
            visualize_reachability_results(result_folder, ...
                'plotType', plot_types, ...
                'saveFigs', save_plots, ...
                'figFormat', 'png');
        else
            % Use legacy visualization functions
            if strcmp(direction, 'backward')
                visualize_brs_results(result_folder, ...
                    'plotType', plot_types, ...
                    'saveFigs', save_plots, ...
                    'figFormat', 'png');
            else
                visualize_frs_results(result_folder, ...
                    'plotType', plot_types, ...
                    'saveFigs', save_plots, ...
                    'figFormat', 'png');
            end
        end
    else
        % For NonlinearBicycleSteered model
        if strcmp(direction, 'backward')
            visualize_brs_results_steered(result_folder, ...
                'plotType', plot_types, ...
                'saveFigs', save_plots, ...
                'figFormat', 'png');
        else
            visualize_frs_results_steered(result_folder, ...
                'plotType', plot_types, ...
                'saveFigs', save_plots, ...
                'figFormat', 'png');
        end
        
        % Generate 3D visualization if requested
        if show_3d_viz
            visualize_3d_reachable_sets(result_folder);
        end
    end
    
    fprintf('Visualization complete.\n');
end

fprintf('All processing completed successfully!\n');
if ~isempty(result_folder)
    fprintf('Results saved to: %s\n', result_folder);
end

end