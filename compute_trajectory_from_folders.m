function [traj, traj_tau, traj_u, traj_metrics] = compute_trajectory_from_folders(brs_folder, xinit, varargin)
% COMPUTE_TRAJECTORY_FROM_FOLDERS Compute optimal trajectory directly from result folders
%
% This wrapper function loads data from BRS and optionally FRS result folders
% and computes the optimal trajectory for a given initial state, targeting the
% original target set used in BRS computation or a custom target set.
%
% Inputs:
%   brs_folder  - Path to the folder containing BRS computation results
%   xinit       - Initial state [beta; gamma] (sideslip angle and yaw rate) in radians
%   varargin    - Optional parameters as name-value pairs:
%                 'useTargetSet' - Use the original target set from BRS results (default: true)
%                 'finalSet'    - Value function defining custom target set (optional)
%                 'customTargetSize' - Create a custom target set with specified size [rad_beta, rad_gamma]
%                 'targetCenter' - Center of custom target set (default: [0, 0])
%                 'frs_folder'  - Path to FRS results folder (optional)
%                 'useFRS'      - Whether to use FRS data (default: false)
%                 'frsWeight'   - Weight for FRS constraint (0-1, default: 0.5)
%                 'finalSetWeight' - Weight for final set targeting (default: 0.7)
%                 'velocityIdx' - Index of velocity to use (default: 1)
%                 'mzMaxIdx'    - Index of yaw moment limit to use (default: 1)
%                 'uMode'       - Control mode ('min' for BRS, default)
%                 'visualize'   - Whether to visualize (default: true)
%                 'savePlots'   - Whether to save plots (default: false)
%                 'figNum'      - Figure number for visualization
%                 'maxTime'     - Maximum trajectory time (seconds, default: based on BRS)
%
% Outputs:
%   traj        - Optimal trajectory states [beta; gamma] over time
%   traj_tau    - Time points corresponding to trajectory
%   traj_u      - Control inputs along trajectory
%   traj_metrics- Structure with additional metrics (if requested)
%
% Example:
%   % Using original target set from BRS computation:
%   [traj, ~, ~] = compute_trajectory_from_folders('path/to/brs_results', [0.1; 0.2]);
%
%   % Using a custom target set:
%   [traj, ~, ~] = compute_trajectory_from_folders('path/to/brs_results', [0.1; 0.2], ...
%       'useTargetSet', false, 'customTargetSize', [deg2rad(1), deg2rad(2)]);
%
%   % With FRS safety constraints:
%   [traj, ~, ~] = compute_trajectory_from_folders('path/to/brs_results', [0.1; 0.2], ...
%       'frs_folder', 'path/to/frs_results', 'useFRS', true);

%% Parse inputs
p = inputParser;
p.addRequired('brs_folder', @ischar);
p.addRequired('xinit', @isnumeric);
p.addParameter('useTargetSet', true, @islogical);
p.addParameter('finalSet', [], @(x) isempty(x) || isnumeric(x));
p.addParameter('customTargetSize', [], @(x) isempty(x) || (isnumeric(x) && length(x) == 2));
p.addParameter('targetCenter', [0, 0], @(x) isnumeric(x) && length(x) == 2);
p.addParameter('frs_folder', '', @ischar);
p.addParameter('useFRS', false, @islogical);
p.addParameter('frsWeight', 0.5, @(x) isnumeric(x) && x >= 0 && x <= 1);
p.addParameter('finalSetWeight', 0.7, @(x) isnumeric(x) && x >= 0 && x <= 1);
p.addParameter('velocityIdx', 1, @isnumeric);
p.addParameter('mzMaxIdx', 1, @isnumeric);
p.addParameter('uMode', 'min', @ischar);
p.addParameter('visualize', true, @islogical);
p.addParameter('savePlots', false, @islogical);
p.addParameter('figNum', [], @(x) isempty(x) || isnumeric(x));
p.addParameter('output_folder', '', @ischar);
p.addParameter('maxTime', [], @(x) isempty(x) || (isnumeric(x) && x > 0));

p.parse(brs_folder, xinit, varargin{:});
opts = p.Results;

% Check for FRS folder if using FRS
if opts.useFRS && isempty(opts.frs_folder)
    error('frs_folder must be provided when useFRS is true');
end

%% Create output folder if saving plots
if opts.savePlots
    if isempty(opts.output_folder)
        opts.output_folder = fullfile(pwd, 'trajectory_results');
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
mzmax_values = brs_data.mzmax_values;
tau_brs = brs_data.tau;
base_params = brs_data.base_params;
data0 = brs_data.data0;  % This is the original target set

% Check valid indices
if opts.velocityIdx > length(velocities)
    error('velocityIdx exceeds available velocities');
end
if opts.mzMaxIdx > length(mzmax_values)
    error('mzMaxIdx exceeds available Mzmax values');
end

% Extract specific BRS data for the selected indices
data_brs = brs_data.all_data{opts.velocityIdx, opts.mzMaxIdx};
data_brs_full = brs_data.all_data_full{opts.velocityIdx, opts.dvMaxIdx};

% Update vehicle parameters with selected velocity and Mzmax
params = base_params;
params(2) = velocities(opts.velocityIdx);  % Update velocity
params(7) = mzmax_values(opts.mzMaxIdx);   % Update Mzmax
params(8) = -mzmax_values(opts.mzMaxIdx);  % Update Mzmin

fprintf('Using velocity = %d m/s, Mzmax = %d N·m\n', ...
    velocities(opts.velocityIdx), mzmax_values(opts.mzMaxIdx));

%% Determine which target set to use
if ~isempty(opts.finalSet)
    % Use explicitly provided final set
    finalSet = opts.finalSet;
    fprintf('Using provided custom target set.\n');
elseif ~isempty(opts.customTargetSize)
    % Create a custom target set with specified size and center
    fprintf('Creating custom target set with size [%.2f°, %.2f°/s] centered at [%.2f°, %.2f°/s]...\n', ...
        opts.customTargetSize(1)*180/pi, opts.customTargetSize(2)*180/pi, ...
        opts.targetCenter(1)*180/pi, opts.targetCenter(2)*180/pi);
    
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
    % Use the BRS itself as the target (backward compatible behavior)
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
    
    % Find matching velocity and Mzmax in FRS data
    v_idx = find(frs_data.velocities == velocities(opts.velocityIdx), 1);
    m_idx = find(frs_data.mzmax_values == mzmax_values(opts.mzMaxIdx), 1);
    
    if isempty(v_idx) || isempty(m_idx)
        warning('Could not find matching velocity or Mzmax in FRS data. Using closest available.');
        
        % Find closest velocity
        [~, v_idx] = min(abs(frs_data.velocities - velocities(opts.velocityIdx)));
        
        % Find closest Mzmax
        [~, m_idx] = min(abs(frs_data.mzmax_values - mzmax_values(opts.mzMaxIdx)));
    end
    
    % Extract FRS data
    data_frs = frs_data.all_data{v_idx, m_idx};
    tau_frs = frs_data.tau;
    
    fprintf('Using FRS data with velocity = %d m/s, Mzmax = %d N·m\n', ...
        frs_data.velocities(v_idx), frs_data.mzmax_values(m_idx));
end

%% Check if initial state is in BRS
initial_value_brs = eval_u(g, data_brs(:,:,end), xinit);
if initial_value_brs > 0
    error(['Initial state [', num2str(xinit(1) * 180/pi, '%.2f'), '°, ', ...
           num2str(xinit(2) * 180/pi, '%.2f'), '°/s] is not in the BRS (value = ', ...
           num2str(initial_value_brs), ').']);
end

% If final set is provided, check if it intersects with the BRS
if ~isempty(finalSet)
    fprintf('Verifying intersection between BRS and target set...\n');
    
    % Sample some points from the final set to check if any are in the BRS
    in_final_set = (finalSet <= 0);
    in_brs = (data_brs(:,:,end) <= 0);
    
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
    initial_value_frs = eval_u(g, data_frs(:,:,end), xinit);
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

extra_args.maxTime = extra_args.maxTime;
% Call trajectory computation function
[traj, traj_tau, traj_u, traj_metrics] = compute_optimal_trajectory(g, data_brs, tau_brs, xinit, params, extra_args);

%% Save results if requested
if opts.savePlots
    % Create descriptive filename
    v_str = num2str(velocities(opts.velocityIdx));
    mz_str = num2str(mzmax_values(opts.mzMaxIdx));
    init_str = [num2str(xinit(1) * 180/pi, '%.1f'), '_', num2str(xinit(2) * 180/pi, '%.1f')];
    
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
    base_filename = ['traj_v', v_str, '_mz', mz_str, '_init', init_str, target_str, '_', timestamp];
    
    % Save figure
    if opts.visualize
        fig_filename = fullfile(opts.output_folder, [base_filename, '.png']);
        saveas(gcf, fig_filename);
        fprintf('Saved figure to %s\n', fig_filename);
    end
    
    % Save data
    data_filename = fullfile(opts.output_folder, [base_filename, '.mat']);
    save(data_filename, 'traj', 'traj_tau', 'traj_u', 'traj_metrics', ...
         'xinit', 'velocities', 'mzmax_values', 'finalSet', 'opts');
    fprintf('Saved trajectory data to %s\n', data_filename);
end

%% Display summary
fprintf('\nTrajectory Computation Summary:\n');
fprintf('--------------------------------\n');
fprintf('Initial state: [%.2f°, %.2f°/s]\n', xinit(1)*180/pi, xinit(2)*180/pi);

if ~isempty(finalSet)
    if opts.useTargetSet
        fprintf('Using original target set from BRS computation\n');
    elseif ~isempty(opts.customTargetSize)
        fprintf('Using custom target set with size [%.2f°, %.2f°/s]\n', ...
            opts.customTargetSize(1)*180/pi, opts.customTargetSize(2)*180/pi);
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

fprintf('Final state: [%.2f°, %.2f°/s]\n', traj(1,end)*180/pi, traj(2,end)*180/pi);
fprintf('Trajectory time: %.2f seconds\n', traj_metrics.time_to_target);
fprintf('Max sideslip angle: %.2f degrees\n', traj_metrics.max_beta*180/pi);
fprintf('Max yaw rate: %.2f degrees/s\n', traj_metrics.max_gamma*180/pi);
fprintf('Max control input: %.2f kN·m\n', traj_metrics.max_control/1000);
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