function [traj, traj_tau, traj_u, traj_metrics] = compute_trajectory_steered_from_folders(brs_folder, xinit, varargin)
% COMPUTE_TRAJECTORY_STEERED_FROM_FOLDERS Compute optimal trajectory for steered bicycle model
%
% This wrapper function loads data from BRS and optionally FRS result folders
% and computes the optimal trajectory for a given initial state, targeting the
% original target set used in BRS computation or a custom target set.
%
% Inputs:
%   brs_folder  - Path to the folder containing BRS computation results
%   xinit       - Initial state [gamma; beta; delta] (yaw rate, sideslip angle, steering angle) in radians
%   varargin    - Optional parameters as name-value pairs:
%                 'useTargetSet' - Use the original target set from BRS results (default: true)
%                 'finalSet'    - Value function defining custom target set (optional)
%                 'customTargetSize' - Create a custom target set with specified size [rad_gamma, rad_beta, rad_delta]
%                 'targetCenter' - Center of custom target set (default: [0, 0, 0])
%                 'frs_folder'  - Path to FRS results folder (optional)
%                 'useFRS'      - Whether to use FRS data (default: false)
%                 'frsWeight'   - Weight for FRS constraint (0-1, default: 0.5)
%                 'finalSetWeight' - Weight for final set targeting (default: 0.7)
%                 'velocityIdx' - Index of velocity to use (default: 1)
%                 'dvMaxIdx'    - Index of steering rate limit to use (default: 1)
%                 'uMode'       - Control mode ('min' for BRS, default)
%                 'visualize'   - Whether to visualize (default: true)
%                 'savePlots'   - Whether to save plots (default: false)
%                 'figNum'      - Figure number for visualization
%                 'maxTime'     - Maximum trajectory time (seconds, default: based on BRS)
%                 'deltaSlice'  - Which delta slice to visualize for 2D views (default: middle slice)
%
% Outputs:
%   traj        - Optimal trajectory states [gamma; beta; delta] over time
%   traj_tau    - Time points corresponding to trajectory
%   traj_u      - Control inputs (steering rate) along trajectory
%   traj_metrics- Structure with additional metrics
%
% Example:
%   % Using original target set from BRS computation:
%   [traj, ~, ~] = compute_trajectory_steered_from_folders('path/to/brs_results', 
%                   [deg2rad(5); deg2rad(2); deg2rad(3)]);
%
%   % Using a custom target set:
%   [traj, ~, ~] = compute_trajectory_steered_from_folders('path/to/brs_results', 
%                   [deg2rad(5); deg2rad(2); deg2rad(3)],
%                   'useTargetSet', false, 
%                   'customTargetSize', [deg2rad(2), deg2rad(1), deg2rad(0.5)]);

%% Parse inputs
p = inputParser;
p.addRequired('brs_folder', @ischar);
p.addRequired('xinit', @isnumeric);
p.addParameter('useTargetSet', true, @islogical);
p.addParameter('finalSet', [], @(x) isempty(x) || isnumeric(x));
p.addParameter('customTargetSize', [], @(x) isempty(x) || (isnumeric(x) && length(x) == 3));
p.addParameter('targetCenter', [0, 0, 0], @(x) isnumeric(x) && length(x) == 3);
p.addParameter('frs_folder', '', @ischar);
p.addParameter('useFRS', false, @islogical);
p.addParameter('frsWeight', 0.5, @(x) isnumeric(x) && x >= 0 && x <= 1);
p.addParameter('finalSetWeight', 0.7, @(x) isnumeric(x) && x >= 0 && x <= 1);
p.addParameter('velocityIdx', 1, @isnumeric);
p.addParameter('dvMaxIdx', 1, @isnumeric);
p.addParameter('uMode', 'min', @ischar);
p.addParameter('visualize', true, @islogical);
p.addParameter('savePlots', false, @islogical);
p.addParameter('figNum', [], @(x) isempty(x) || isnumeric(x));
p.addParameter('output_folder', '', @ischar);
p.addParameter('maxTime', [], @(x) isempty(x) || (isnumeric(x) && x > 0));
p.addParameter('deltaSlice', [], @(x) isempty(x) || isnumeric(x));

p.parse(brs_folder, xinit, varargin{:});
opts = p.Results;

% Check for FRS folder if using FRS
if opts.useFRS && isempty(opts.frs_folder)
    error('frs_folder must be provided when useFRS is true');
end

% Check that initial state has proper dimensions
if length(xinit) ~= 3
    error('Initial state must have 3 elements [gamma; beta; delta] for the steered bicycle model');
end

%% Create output folder if saving plots
if opts.savePlots
    if isempty(opts.output_folder)
        opts.output_folder = fullfile(pwd, 'trajectory_steered_results');
    end
    
    if ~exist(opts.output_folder, 'dir')
        mkdir(opts.output_folder);
        fprintf('Created output directory: %s\n', opts.output_folder);
    end
end

%% Load BRS results
fprintf('Loading BRS data from %s...\n', brs_folder);

% Load combined results file
combined_file = fullfile(brs_folder, 'brs_combined_results.mat');
if ~exist(combined_file, 'file')
    error('Combined results file not found: %s', combined_file);
end

brs_data = load(combined_file);

% Extract necessary data
g = brs_data.g;
velocities = brs_data.velocities;
dvmax_values = brs_data.dvmax_values;
tau_brs = brs_data.tau;
base_params = brs_data.base_params;
data0 = brs_data.data0;  % This is the original target set

% Check valid indices
if opts.velocityIdx > length(velocities)
    error('velocityIdx exceeds available velocities');
end
if opts.dvMaxIdx > length(dvmax_values)
    error('dvMaxIdx exceeds available steering rate limits');
end

% Extract specific BRS data for the selected indices
data_brs = brs_data.all_data{opts.velocityIdx, opts.dvMaxIdx};
data_brs_full = brs_data.all_data_full{opts.velocityIdx, opts.dvMaxIdx};

% Update vehicle parameters with selected velocity and steering rate limit
params = base_params;
params(2) = velocities(opts.velocityIdx);  % Update velocity
params(7) = dvmax_values(opts.dvMaxIdx);   % Update dv_max

fprintf('Using velocity = %d m/s, Max Steering Rate = %.2f rad/s (%.2f°/s)\n', ...
    velocities(opts.velocityIdx), dvmax_values(opts.dvMaxIdx), dvmax_values(opts.dvMaxIdx)*180/pi);

%% Determine which target set to use
if ~isempty(opts.finalSet)
    % Use explicitly provided final set
    finalSet = opts.finalSet;
    fprintf('Using provided custom target set.\n');
elseif ~isempty(opts.customTargetSize)
    % Create a custom target set with specified size and center
    fprintf('Creating custom target set with size [%.2f°, %.2f°, %.2f°] centered at [%.2f°, %.2f°, %.2f°]...\n', ...
        opts.customTargetSize(1)*180/pi, opts.customTargetSize(2)*180/pi, opts.customTargetSize(3)*180/pi, ...
        opts.targetCenter(1)*180/pi, opts.targetCenter(2)*180/pi, opts.targetCenter(3)*180/pi);
    
    finalSet = shapeRectangleByCenter(g, opts.targetCenter, opts.customTargetSize);
    
    % Ensure the target set is properly shaped (negative inside, positive outside)
    if eval_u(g, finalSet, opts.targetCenter) > 0
        % If the center is outside the set, flip the sign
        finalSet = -finalSet;
        fprintf('Note: Target set was inverted to ensure correct sign convention.\n');
    end
elseif opts.useTargetSet
    % Use the original target set from BRS computation
    finalSet = data0;
    fprintf('Using original target set from BRS computation.\n');
else
    % Use the BRS itself as the target
    finalSet = [];
    fprintf('Using the BRS itself as the target set.\n');
end

%% Load FRS results if requested
data_frs = [];
tau_frs = [];
if opts.useFRS
    fprintf('Loading FRS data from %s...\n', opts.frs_folder);
    
    % Load combined results file
    frs_combined_file = fullfile(opts.frs_folder, 'frs_combined_results.mat');
    if ~exist(frs_combined_file, 'file')
        % Try alternate filename
        frs_combined_file = fullfile(opts.frs_folder, 'brs_combined_results.mat');
        if ~exist(frs_combined_file, 'file')
            error('FRS combined results file not found in the specified folder');
        end
    end
    
    frs_data = load(frs_combined_file);
    
    % Check compatibility
    if ~isequal(frs_data.g.min, g.min) || ~isequal(frs_data.g.max, g.max) || ~isequal(frs_data.g.N, g.N)
        warning('FRS and BRS grids are not identical. Results may be inconsistent.');
    end
    
    % Find matching velocity and dvmax in FRS data
    v_idx = find(frs_data.velocities == velocities(opts.velocityIdx), 1);
    dv_idx = find(frs_data.dvmax_values == dvmax_values(opts.dvMaxIdx), 1);
    
    if isempty(v_idx) || isempty(dv_idx)
        warning('Could not find matching velocity or dvmax in FRS data. Using closest available.');
        
        % Find closest velocity
        [~, v_idx] = min(abs(frs_data.velocities - velocities(opts.velocityIdx)));
        
        % Find closest dvmax
        [~, dv_idx] = min(abs(frs_data.dvmax_values - dvmax_values(opts.dvMaxIdx)));
    end
    
    % Extract FRS data
    data_frs = frs_data.all_data{v_idx, dv_idx};
    tau_frs = frs_data.tau;
    
    fprintf('Using FRS data with velocity = %d m/s, Max Steering Rate = %.2f rad/s (%.2f°/s)\n', ...
        frs_data.velocities(v_idx), frs_data.dvmax_values(dv_idx), frs_data.dvmax_values(dv_idx)*180/pi);
end

%% Check if initial state is in BRS
initial_value_brs = eval_u(g, data_brs, xinit);
if initial_value_brs > 0
    error(['Initial state [', num2str(xinit(1) * 180/pi, '%.2f'), '°/s, ', ...
           num2str(xinit(2) * 180/pi, '%.2f'), '°, ', ...
           num2str(xinit(3) * 180/pi, '%.2f'), '°] is not in the BRS (value = ', ...
           num2str(initial_value_brs), ').']);
end

% If final set is provided, check if it intersects with the BRS
if ~isempty(finalSet)
    fprintf('Verifying intersection between BRS and target set...\n');
    
    % Sample some points from the final set to check if any are in the BRS
    in_final_set = (finalSet <= 0);
    in_brs = (data_brs <= 0);
    
    % Check if there's an intersection
    intersection = in_final_set & in_brs;
    if ~any(intersection(:))
        warning('The target set does not intersect with the BRS. The trajectory may not reach the target.');
    else
        fprintf('Target set intersects with BRS. Trajectory planning should be feasible.\n');
    end
end

% Check if initial state is in FRS if available
if opts.useFRS && ~isempty(data_frs)
    initial_value_frs = eval_u(g, data_frs, xinit);
    if initial_value_frs > 0
        warning(['Initial state is not in the FRS (value = ', num2str(initial_value_frs), ').']);
    else
        fprintf('Initial state is in both BRS and FRS.\n');
    end
end

%% Compute optimal trajectory
fprintf('Computing optimal trajectory...\n');

% Setup additional arguments
extra_args = struct();
extra_args.uMode = opts.uMode;
extra_args.visualize = opts.visualize;
extra_args.figNum = opts.figNum;
extra_args.deltaSlice = opts.deltaSlice;

if ~isempty(finalSet)
    extra_args.finalSet = finalSet;
    extra_args.finalSetWeight = opts.finalSetWeight;
end

if ~isempty(opts.maxTime)
    extra_args.maxTime = opts.maxTime;
end

if opts.useFRS && ~isempty(data_frs)
    extra_args.data_frs = data_frs;
    extra_args.tau_frs = tau_frs;
    extra_args.useFRSConstraint = true;
    extra_args.frsWeight = opts.frsWeight;
end

%% Compute time-of-arrival function for better gradients
fprintf('Computing time-of-arrival function for more robust trajectory planning...\n');

% Extract data for all time steps if available (for 3D state)
if ndims(data_brs) == 3  % Only final time slice is provided
    if isfield(extra_args, 'all_data_full') && ~isempty(extra_args.all_data_full)
        % Use full time data if provided in extra_args
        data_full = extra_args.all_data_full;
    else
        % Otherwise, use the single time slice (less accurate but still workable)
        fprintf('Warning: Using only final time slice. Trajectory may be suboptimal.\n');
        data_full = data_brs;
    end
else
    % Data already contains all time steps
    data_full = data_brs;
end

% Compute time-of-arrival function
arrival_time = compute_arrival_time(data_brs_full, tau_brs);

% Check if initial state is in BRS using time-of-arrival function
arrival_val = eval_u(g, arrival_time, xinit);
if ~isfinite(arrival_val)
    warning('Initial state is not in the BRS according to time-of-arrival function.');
else
    fprintf('Initial state is reachable in %.2f seconds.\n', arrival_val);
end

% Add the time-of-arrival function to extra_args
extra_args.arrival_time = arrival_time;

% Extract xinit and params from dCar
[traj, traj_tau, traj_u, traj_metrics] = compute_optimal_trajectory_steered(g, data_brs, tau_brs, xinit, params, extra_args);

%% Save results if requested
if opts.savePlots
    % Create descriptive filename
    v_str = num2str(velocities(opts.velocityIdx));
    dv_str = num2str(round(dvmax_values(opts.dvMaxIdx)*180/pi));
    init_str = [num2str(xinit(1) * 180/pi, '%.1f'), '_', num2str(xinit(2) * 180/pi, '%.1f'), '_', num2str(xinit(3) * 180/pi, '%.1f')];
    
    if ~isempty(finalSet)
        if opts.useTargetSet
            target_str = '_to_originalTarget';
        else
            target_str = '_to_customTarget';
        end
    else
        target_str = '';
    end
    
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    base_filename = ['steered_traj_v', v_str, '_dv', dv_str, '_init', init_str, target_str, '_', timestamp];
    
    % Save figure
    if opts.visualize
        fig_filename = fullfile(opts.output_folder, [base_filename, '.png']);
        saveas(gcf, fig_filename);
        fprintf('Saved figure to %s\n', fig_filename);
    end
    
    % Save data
    data_filename = fullfile(opts.output_folder, [base_filename, '.mat']);
    save(data_filename, 'traj', 'traj_tau', 'traj_u', 'traj_metrics', ...
         'xinit', 'velocities', 'dvmax_values', 'finalSet', 'opts');
    fprintf('Saved trajectory data to %s\n', data_filename);
end

%% Display summary
fprintf('\nTrajectory Computation Summary:\n');
fprintf('--------------------------------\n');
fprintf('Initial state: [%.2f°/s, %.2f°, %.2f°]\n', xinit(1)*180/pi, xinit(2)*180/pi, xinit(3)*180/pi);

if ~isempty(finalSet)
    if opts.useTargetSet
        fprintf('Using original target set from BRS computation\n');
    elseif ~isempty(opts.customTargetSize)
        fprintf('Using custom target set with size [%.2f°, %.2f°, %.2f°]\n', ...
            opts.customTargetSize(1)*180/pi, opts.customTargetSize(2)*180/pi, opts.customTargetSize(3)*180/pi);
    else
        fprintf('Using provided custom target set\n');
    end
    
    if isfield(traj_metrics, 'final_set_value')
        if traj_metrics.final_set_value <= 0
            fprintf('Target set reached! (final value: %.4f)\n', traj_metrics.final_set_value);
        else
            fprintf('Target set NOT reached (final value: %.4f)\n', traj_metrics.final_set_value);
        end
    end
    
    if isfield(traj_metrics, 'reached_target')
        if traj_metrics.reached_target
            fprintf('Target successfully reached!\n');
        else
            fprintf('Target not reached within tolerance.\n');
        end
    end
else
    fprintf('Using the BRS itself as the target set\n');
end

fprintf('Final state: [%.2f°/s, %.2f°, %.2f°]\n', traj(1,end)*180/pi, traj(2,end)*180/pi, traj(3,end)*180/pi);
fprintf('Trajectory time: %.2f seconds\n', traj_metrics.time_to_target);
fprintf('Max yaw rate: %.2f degrees/s\n', traj_metrics.max_gamma*180/pi);
fprintf('Max sideslip angle: %.2f degrees\n', traj_metrics.max_beta*180/pi);
fprintf('Max steering angle: %.2f degrees\n', traj_metrics.max_delta*180/pi);
fprintf('Max control input: %.2f degrees/s\n', traj_metrics.max_control*180/pi);
fprintf('Control energy: %.2e\n', traj_metrics.control_energy);

if opts.useFRS && isfield(traj_metrics, 'in_frs')
    if traj_metrics.in_frs
        fprintf('Trajectory stays within FRS bounds (safe)\n');
    else
        fprintf('WARNING: Trajectory violates FRS bounds at %.2f seconds\n', traj_metrics.frs_violation_time);
    end
end

fprintf('--------------------------------\n');
end