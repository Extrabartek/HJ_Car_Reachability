function verify_reachability_results(results_folder, trajectory_file, varargin)
% VERIFY_REACHABILITY_RESULTS Unified wrapper for verifying reachability results
%
% This function automatically detects the model type (Double Integrator or 
% Dubins Car) and runs the appropriate verification functions to analyze both
% reachable sets and optimal trajectories.
%
% Inputs:
%   results_folder   - Path to the reachability results folder
%   trajectory_file  - Path to the trajectory file (or empty to skip trajectory verification)
%   varargin         - Optional parameter-value pairs:
%     'visualize'     - Whether to show plots (default: true)
%     'save_figures'  - Whether to save figures (default: false)
%     'output_folder' - Folder to save verification results (default: results_folder/verification)
%     'time_index'    - Time index to analyze (default: final time)
%     'control_limit' - Control limit (override detection from data)
%
% Example:
%   verify_reachability_results('results/doubleint_brs_results_20250507', ...
%                              'trajectory_data.mat');

%% Parse inputs
p = inputParser;
p.addRequired('results_folder', @ischar);
p.addRequired('trajectory_file', @(x) ischar(x) || isempty(x));
p.addParameter('visualize', true, @islogical);
p.addParameter('save_figures', false, @islogical);
p.addParameter('output_folder', '', @ischar);
p.addParameter('time_index', [], @isnumeric);
p.addParameter('control_limit', [], @isnumeric);

result_folder = '/home/bartosz/Documents/master_thesis/code_base/HJ_Car_Reachability/results/';
whole_path = strcat(result_folder, results_folder);

p.parse(results_folder, trajectory_file, varargin{:});
opts = p.Results;

results_folder = whole_path;

% Set up output folder
if isempty(opts.output_folder)
    opts.output_folder = fullfile(results_folder, 'verification');
end

if opts.save_figures && ~exist(opts.output_folder, 'dir')
    mkdir(opts.output_folder);
    fprintf('Created output directory: %s\n', opts.output_folder);
end

%% Check if folders exist
if ~exist(results_folder, 'dir')
    error('Results folder not found: %s', results_folder);
end

%% Determine computation type (BRS or FRS)
computation_type = '';
% Check folder name
if contains(lower(results_folder), 'brs')
    computation_type = 'brs';
elseif contains(lower(results_folder), 'frs')
    computation_type = 'frs';
end

% If not found in folder name, check for files
if isempty(computation_type)
    if exist(fullfile(results_folder, 'brs_combined_results.mat'), 'file')
        computation_type = 'brs';
    elseif exist(fullfile(results_folder, 'frs_combined_results.mat'), 'file')
        computation_type = 'frs';
    end
end

% If still not determined, prompt the user
if isempty(computation_type)
    user_input = input('Could not determine computation type. Is this a BRS or FRS? [B/F]: ', 's');
    if strcmpi(user_input, 'B') || strcmpi(user_input, 'BRS')
        computation_type = 'brs';
    elseif strcmpi(user_input, 'F') || strcmpi(user_input, 'FRS')
        computation_type = 'frs';
    else
        error('Invalid input. Please specify "B" or "F".');
    end
end

fprintf('Detected computation type: %s\n', upper(computation_type));

%% Load the data to determine model type
fprintf('Loading data to determine model type...\n');

% Try to load combined results file
combined_file = fullfile(results_folder, [computation_type, '_combined_results.mat']);
if ~exist(combined_file, 'file')
    % Try to find any relevant .mat file
    files = dir(fullfile(results_folder, [computation_type, '_*.mat']));
    if isempty(files)
        error('No %s data files found in the specified folder', upper(computation_type));
    end
    data_file = fullfile(results_folder, files(1).name);
    fprintf('Loading individual file: %s\n', data_file);
    data = load(data_file);
else
    data = load(combined_file);
end

%% Determine model type
model_type = '';

% First check if model type is explicitly stored
if isfield(data, 'modelType')
    model_type = data.modelType;
elseif isfield(data, 'sim_params') && isfield(data.sim_params, 'modelType')
    model_type = data.sim_params.modelType;
else
    % Try to infer from data structure and dimensions
    if isfield(data, 'g')
        grid_dims = length(data.g.N);
        
        if grid_dims == 2
            % Could be Double Integrator or 2D simplified Dubins
            if isfield(data, 'acceleration_limits') || ...
               (isfield(data, 'sim_params') && isfield(data.sim_params, 'acceleration_limits'))
                model_type = 'doubleInt';
            else
                % Check folder name for hints
                if contains(lower(results_folder), 'double') || ...
                   contains(lower(results_folder), 'integrator')
                    model_type = 'doubleInt';
                elseif contains(lower(results_folder), 'dubins') || ...
                       contains(lower(results_folder), 'car')
                    model_type = 'dubinsCar';
                end
            end
        elseif grid_dims == 3
            % Could be 3D Dubins Car or 3D steered bicycle
            if isfield(data, 'turning_limits') || ...
               (isfield(data, 'sim_params') && isfield(data.sim_params, 'turning_limits')) || ...
               isfield(data, 'speed')
                model_type = 'dubinsCar';
            elseif isfield(data, 'dvmax_values') || isfield(data, 'mzmax_values')
                model_type = 'bicycle';
            else
                % Check folder name for hints
                if contains(lower(results_folder), 'dubins') || ...
                   contains(lower(results_folder), 'car')
                    model_type = 'dubinsCar';
                elseif contains(lower(results_folder), 'bicycle') || ...
                       contains(lower(results_folder), 'steered')
                    model_type = 'bicycle';
                end
            end
        end
    end
end

% If still not determined, prompt the user
if isempty(model_type)
    fprintf('Could not automatically determine model type.\n');
    fprintf('1. Double Integrator\n');
    fprintf('2. Dubins Car\n');
    fprintf('3. Bicycle Model\n');
    user_input = input('Please select model type [1-3]: ');
    
    switch user_input
        case 1
            model_type = 'doubleInt';
        case 2
            model_type = 'dubinsCar';
        case 3
            model_type = 'bicycle';
        otherwise
            error('Invalid selection');
    end
end

fprintf('Detected model type: %s\n', model_type);

%% Extract key parameters
% Get control limit if provided or extract from data
control_limit = opts.control_limit;

if isempty(control_limit)
    % Try to extract from data based on model type
    if strcmp(model_type, 'doubleInt')
        if isfield(data, 'acceleration_limits')
            control_limit = data.acceleration_limits(1);
        elseif isfield(data, 'sim_params') && isfield(data.sim_params, 'acceleration_limits')
            control_limit = data.sim_params.acceleration_limits(1);
        end
    elseif strcmp(model_type, 'dubinsCar')
        if isfield(data, 'turning_limits')
            control_limit = data.turning_limits(1);
        elseif isfield(data, 'sim_params') && isfield(data.sim_params, 'turning_limits')
            control_limit = data.sim_params.turning_limits(1);
        end
    elseif strcmp(model_type, 'bicycle')
        if isfield(data, 'controlType')
            if strcmp(data.controlType, 'mz') && isfield(data, 'mzmax_values')
                control_limit = data.mzmax_values(1);
            elseif strcmp(data.controlType, 'dv') && isfield(data, 'dvmax_values')
                control_limit = data.dvmax_values(1);
            end
        elseif isfield(data, 'mzmax_values')
            control_limit = data.mzmax_values(1);
        elseif isfield(data, 'dvmax_values')
            control_limit = data.dvmax_values(1);
        end
    end
    
    if isempty(control_limit)
        warning('Could not determine control limit from data.');
    else
        fprintf('Extracted control limit: %.2f\n', control_limit);
    end
end

%% Create verification parameters
verification_params = struct();
verification_params.visualize = opts.visualize;
verification_params.save_figures = opts.save_figures;
verification_params.output_folder = opts.output_folder;
verification_params.time_index = opts.time_index;
verification_params.control_limit = control_limit;

%% Run appropriate verification function for reachable set
fprintf('\n=== Verifying %s Reachable Set ===\n', upper(computation_type));

verification_results = struct();

if strcmp(model_type, 'doubleInt')
    % Verify Double Integrator
        % Check if the verification function exists
        if exist('verify_double_integrator_brs', 'file')
            [rms_error, r_squared] = verify_double_integrator_brs(results_folder, ...
                'control_limit', control_limit, ...
                'time_index', opts.time_index, ...
                'visualize', opts.visualize);
            
            verification_results.rms_error = rms_error;
            verification_results.r_squared = r_squared;
            
            % Save the figure if requested
            if opts.save_figures && opts.visualize
                fig_handle = findobj('Type', 'figure', 'Name', 'Double Integrator BRS Verification');
                if ~isempty(fig_handle)
                    saveas(fig_handle, fullfile(opts.output_folder, 'double_integrator_verification.png'));
                    fprintf('Saved verification figure to %s\n', fullfile(opts.output_folder, 'double_integrator_verification.png'));
                end
            end
        else
            error('verify_double_integrator_brs function not found. Please add it to your MATLAB path.');
        end
    
elseif strcmp(model_type, 'dubinsCar')
    % Verify Dubins Car
    try
        % Check if the verification function exists
        if exist('verify_dubins_car_properties', 'file')
            verify_dubins_car_properties(results_folder, verification_params);
            
            % Save the figure if requested
            if opts.save_figures && opts.visualize
                fig_handle = findobj('Type', 'figure', 'Name', 'Dubins Car Verification');
                if ~isempty(fig_handle)
                    saveas(fig_handle, fullfile(opts.output_folder, 'dubins_car_verification.png'));
                    fprintf('Saved verification figure to %s\n', fullfile(opts.output_folder, 'dubins_car_verification.png'));
                end
            end
        else
            error('verify_dubins_car_properties function not found. Please add it to your MATLAB path.');
        end
    catch err
        warning('Error in Dubins car verification: %s', err.message);
    end
    
elseif strcmp(model_type, 'bicycle')
    % For bicycle model, we don't have a direct analytical solution
    % but we can check for expected properties
    fprintf('Bicycle model verification is not implemented yet.\n');
    fprintf('Skipping reachable set verification for bicycle model.\n');
end

%% Verify trajectory if a trajectory file is provided
if ~isempty(trajectory_file) && exist(trajectory_file, 'file')
    fprintf('\n=== Verifying Optimal Trajectory ===\n');
    
    % Load the trajectory
    try
        traj_data = load(trajectory_file);
        
        % Check if the required variables exist
        required_vars = {'traj', 'traj_tau', 'traj_u'};
        missing_vars = setdiff(required_vars, fieldnames(traj_data));
        
        if ~isempty(missing_vars)
            error('Trajectory file is missing required variables: %s', strjoin(missing_vars, ', '));
        end
        
        % Get trajectory type
        if isfield(traj_data, 'trajectory_type')
            trajectory_type = traj_data.trajectory_type;
        else
            % Try to infer from filename
            if contains(lower(trajectory_file), 'brs')
                trajectory_type = 'brs';
            elseif contains(lower(trajectory_file), 'frs')
                trajectory_type = 'frs';
            else
                % Default to same as computation_type
                trajectory_type = computation_type;
            end
        end
        
        % Get model type from trajectory file, or use the one detected earlier
        if isfield(traj_data, 'model_type')
            traj_model_type = traj_data.model_type;
        else
            traj_model_type = model_type;
        end
        
        fprintf('Trajectory type: %s, Model type: %s\n', ...
            upper(trajectory_type), traj_model_type);
        
        % Check if the verification function exists
        if exist('verify_optimal_trajectory', 'file')
            % Verify the trajectory
            verify_optimal_trajectory(traj_data.traj, traj_data.traj_tau, traj_data.traj_u, ...
                trajectory_type, traj_model_type);
            
            % Save the figure if requested
            if opts.save_figures && opts.visualize
                fig_handle = findobj('Type', 'figure', 'Name', sprintf('%s Trajectory Control Analysis', upper(trajectory_type)));
                if ~isempty(fig_handle)
                    saveas(fig_handle, fullfile(opts.output_folder, 'trajectory_verification.png'));
                    fprintf('Saved trajectory verification figure to %s\n', fullfile(opts.output_folder, 'trajectory_verification.png'));
                end
            end
        else
            error('verify_optimal_trajectory function not found. Please add it to your MATLAB path.');
        end
    catch err
        warning('Error in trajectory verification: %s', err.message);
    end
else
    fprintf('\nNo trajectory file provided or file not found. Skipping trajectory verification.\n');
end

%% Print overall summary
fprintf('\n=== Verification Summary ===\n');
fprintf('Model: %s\n', model_type);
fprintf('Computation Type: %s\n', upper(computation_type));

if strcmp(model_type, 'doubleInt') && isfield(verification_results, 'r_squared')
    fprintf('Reachable Set Verification:\n');
    fprintf('  RMS Error: %.4f\n', verification_results.rms_error);
    fprintf('  R-squared: %.4f\n', verification_results.r_squared);
    
    if verification_results.r_squared > 0.95
        fprintf('  ✅ Analytical verification PASSED (R² > 0.95)\n');
    else
        fprintf('  ❌ Analytical verification FAILED (R² < 0.95)\n');
    end
end

fprintf('\nVerification process completed.\n');

end