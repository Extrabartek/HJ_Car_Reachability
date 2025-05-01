function [traj, traj_tau, traj_u, metrics] = computeOptimizedTrajectory(g, data, tau, xinit, dynSys, options)
% COMPUTEOPTIMIZEDTRAJECTORY Computes optimal trajectory using optimized methods
%
% This function implements optimized methods for computing trajectories based on
% reachability analysis results, supporting both BRS and FRS trajectories.
%
% For BRS trajectories: Control tries to minimize value function to reach target
% For FRS trajectories: Control tries to maximize value function to escape target
%
% Inputs:
%   g           - Grid structure from reachability analysis
%   data        - Value function data (BRS/FRS) (full time series)
%   tau         - Time vector corresponding to the value function data
%   xinit       - Initial state vector
%   dynSys      - Dynamic system object (vehicle model)
%   options     - Structure of options with the following possible fields:
%     .method         - Computation method: 'arrival' or 'gradient' (default: 'gradient')
%     .arrivalTime    - Time-of-arrival function (required if method = 'arrival')
%     .finalSet       - Custom target set (if different from BRS)
%     .useFRS         - Whether to use FRS for safety constraints (default: false)
%     .data_frs       - FRS data (required if useFRS = true)
%     .frsWeight      - Weight for FRS constraints (default: 0.5)
%     .uMode          - Control mode: 'min' (BRS) or 'max' (FRS) (default: 'min')
%     .dMode          - Disturbance mode: 'min' or 'max' (default: 'max')
%     .maxTime        - Maximum trajectory time (default: based on tau)
%     .dt             - Time step for integration (default: derived from tau)
%     .visualize      - Whether to visualize during computation (default: false)
%     .reverseFRS     - For FRS trajectories only: indicates this is an FRS trajectory
%
% Outputs:
%   traj            - Optimal trajectory states (columns are time points)
%   traj_tau        - Time points corresponding to trajectory
%   traj_u          - Control inputs along trajectory
%   metrics         - Structure with performance metrics
%
% Examples:
%   % Using gradient pre-computation method for BRS trajectory:
%   options.method = 'gradient';
%   options.uMode = 'min';
%   [traj, tau] = computeOptimizedTrajectory(g, data, tau, xinit, dynSys, options);
%
%   % Using time-of-arrival function for FRS trajectory:
%   options.method = 'arrival';
%   options.arrivalTime = arrival_time;
%   options.uMode = 'max';  % For FRS, control maximizes value function
%   options.reverseFRS = true;
%   [traj, tau, u] = computeOptimizedTrajectory(g, data, tau, xinit, dynSys, options);

%% Process input parameters and set defaults
if nargin < 6
    options = struct();
end

% Set defaults for options
if ~isfield(options, 'method')
    options.method = 'gradient';
end

if ~isfield(options, 'uMode')
    options.uMode = 'min';  % Default to BRS behavior
end

% Determine if this is an FRS trajectory based on uMode and reverseFRS flag
is_frs_trajectory = strcmp(options.uMode, 'max') || ...
                   (isfield(options, 'reverseFRS') && options.reverseFRS);

if is_frs_trajectory
    % Ensure uMode is set correctly for FRS
    options.uMode = 'max';
    fprintf('FRS trajectory mode: control maximizes value function to escape target\n');
else
    % Ensure uMode is set correctly for BRS
    options.uMode = 'min';
    fprintf('BRS trajectory mode: control minimizes value function to reach target\n');
end

if ~isfield(options, 'dMode')
    options.dMode = 'max';
end

if ~isfield(options, 'useFRS')
    options.useFRS = false;
end

if ~isfield(options, 'frsWeight') && options.useFRS
    options.frsWeight = 0.5;
end

if ~isfield(options, 'maxTime')
    options.maxTime = tau(end);
end

if ~isfield(options, 'dt')
    if length(tau) > 1
        options.dt = (tau(2) - tau(1)) / 10; % Use smaller time step for better accuracy
    else
        options.dt = 0.01; % Default if tau is just a single value
    end
end

if ~isfield(options, 'visualize')
    options.visualize = false;
end

% Validate required parameters based on method
if strcmp(options.method, 'arrival') && ~isfield(options, 'arrivalTime')
    error('Arrival time method specified but options.arrivalTime is missing.');
end

if options.useFRS && ~isfield(options, 'data_frs')
    error('FRS constraints enabled but options.data_frs is missing.');
end

% Get grid dimensions
grid_dims = g.dim;

% Check data dimensionality
data_dims = ndims(data);

% Ensure data is a full time series (if not, try to handle it)
if data_dims == grid_dims
    % Single time slice - expand for backward compatibility
    warning(['Data appears to be a single time slice. ', ...
             'Better results are achieved with full time series data.']);
    data_full = repmat(data, [ones(1, grid_dims), length(tau)]);
else
    % Already full time series
    data_full = data;
end

% For FRS trajectories, we might need to handle time differently
if is_frs_trajectory
    % For FRS, we want to start from t=0 and move forward in time
    % This is similar to BRS computation, but with different control objective
    fprintf('Computing forward trajectory with %s objective\n', options.uMode);
    
    % Check if initial state is close to target set
    if isfield(options, 'finalSet')
        target_value = eval_u(g, options.finalSet, xinit);
        % For FRS trajectories, ideal initial states are inside or close to target
        if target_value > 0
            warning(['For FRS trajectories, initial state should ideally be in the target set. ', ...
                     'Current value: %.4f'], target_value);
        end
    end
end

%% Execute appropriate computation method
switch options.method
    case 'arrival'
        % Method using time-of-arrival function
        [traj, traj_tau, traj_u, metrics] = computeTrajectoryWithArrivalTime(g, data_full, tau, xinit, dynSys, options);
        
    case 'gradient'
        % Optimized method with full gradient precomputation
        [traj, traj_tau, traj_u, metrics] = computeTrajectoryWithGradientPrecomputation(g, data_full, tau, xinit, dynSys, options);
        
    otherwise
        error('Unsupported computation method: %s', options.method);
end

end

%% ===== Trajectory computation with arrival time function =====
function [traj, traj_tau, traj_u, metrics] = computeTrajectoryWithArrivalTime(g, data, tau, xinit, dynSys, options)
% Trajectory computation using time-of-arrival function for better direction

arrival_time = options.arrivalTime;
is_frs_trajectory = strcmp(options.uMode, 'max') || ...
                   (isfield(options, 'reverseFRS') && options.reverseFRS);

% Check if initial state is in reachable set using arrival time function
arr_val = eval_u(g, arrival_time, xinit);
if ~isfinite(arr_val)
    if is_frs_trajectory
        warning('Initial state is not in FRS according to arrival time function.');
    else
        warning('Initial state is not in BRS according to arrival time function.');
    end
    
    % Initialize empty trajectory and metrics
    traj = xinit;
    traj_tau = tau(1);
    traj_u = [];
    
    metrics = struct();
    metrics.time_to_target = 0;
    metrics.final_set_value = inf;
    metrics.reached_target = false;
    metrics.control_energy = 0;
    return;
else
    if is_frs_trajectory
        fprintf('Initial state is in FRS, can be reached in %.2f seconds.\n', arr_val);
    else
        fprintf('Initial state is in BRS, can reach target in %.2f seconds.\n', arr_val);
    end
end

% Compute gradients of arrival time
% For BRS: arrival time DECREASES toward the target, so we negate the gradient
% For FRS: we handle differently based on how arrival time was computed
fprintf('Computing arrival time gradients...\n');
arrival_grad = computeGradients(g, arrival_time);

% For BRS, we negate the gradient to point toward decreasing values (toward target)
if ~is_frs_trajectory
    for i = 1:length(arrival_grad)
        arrival_grad{i} = -arrival_grad{i};  % Negate for BRS
    end
end
% For FRS, if using the standard arrival time computation, we don't negate

% Initialize trajectory
dt = options.dt;
max_time = options.maxTime;
max_steps = ceil(max_time / dt);

traj = zeros(dynSys.nx, max_steps+1);
traj(:,1) = xinit;
traj_tau = (0:max_steps) * dt;
traj_u = zeros(dynSys.nu, max_steps);

% Initialize dynamic system state
dynSys.x = xinit;

% Set up visualization if enabled
if options.visualize
    if is_frs_trajectory
        setupVisualization(g, arrival_time, xinit, 'FRS: Arrival Time Method');
    else
        setupVisualization(g, arrival_time, xinit, 'BRS: Arrival Time Method');
    end
end

% Main trajectory computation loop
for i = 1:max_steps
    % Current state and time
    x_current = traj(:,i);
    t_current = traj_tau(i);
    
    % Get arrival time for current state
    arr_val = eval_u(g, arrival_time, x_current);
    
    % If state is not reachable (arrival_val is inf), stop computation
    if ~isfinite(arr_val)
        warning('Current state at step %d is outside the reachable set. Stopping computation.', i);
        % Truncate trajectory
        traj = traj(:, 1:i);
        traj_tau = traj_tau(1:i);
        traj_u = traj_u(:, 1:i-1);
        break;
    end
    
    % Visualize if enabled
    if options.visualize
        updateVisualization(traj, i, t_current, arr_val);
    end
    
    % Evaluate arrival time gradient at current state
    deriv_at_x = eval_u(g, arrival_grad, x_current);
    
    % Compute control
    u = dynSys.optCtrl(t_current, x_current, deriv_at_x, options.uMode);
    
    % Apply FRS constraint if enabled for BRS trajectories
    if ~is_frs_trajectory && options.useFRS && isfield(options, 'data_frs')
        u = applyFRSConstraint(dynSys, t_current, x_current, deriv_at_x, u, g, options);
    end
    
    % Store control
    traj_u(:,i) = u;
    
    % Check if we've reached our objective based on trajectory type
    if ~is_frs_trajectory
        % For BRS: Check if we've reached the target (very small arrival time)
        if arr_val <= tau(1) + dt  % Very close to initial time = target reached
            fprintf('Target reached at step %d (time %.2fs)!\n', i, t_current);
            % Truncate trajectory
            traj = traj(:, 1:i);
            traj_tau = traj_tau(1:i);
            traj_u = traj_u(:, 1:i-1);
            break;
        end
    else
        % For FRS: Check if we've escaped far enough or reached computation boundary
        % Here we use the value function to check escape, not just arrival time
        if isfield(options, 'finalSet')
            value = eval_u(g, options.finalSet, x_current);
            % If we're significantly outside the target set, consider it escaped
            if value > 0.2  % Use a threshold above 0 to ensure we're well outside
                fprintf('Escaped target sufficiently at step %d (time %.2fs)!\n', i, t_current);
                % Truncate trajectory
                traj = traj(:, 1:i);
                traj_tau = traj_tau(1:i);
                traj_u = traj_u(:, 1:i-1);
                break;
            end
        end
    end
    
    % Propagate dynamics for one step
    if ismethod(dynSys, 'optDstb')
        % With disturbance
        if isfield(options, 'dMode')
            d = dynSys.optDstb(t_current, x_current, deriv_at_x, options.dMode);
        else
            d = dynSys.optDstb(t_current, x_current, deriv_at_x);
        end
        dynSys.updateState(u, dt, x_current, d);
    else
        % Without disturbance
        dynSys.updateState(u, dt, x_current);
    end
    
    traj(:,i+1) = dynSys.x;
    
    % Print progress occasionally
    if mod(i, 100) == 0
        fprintf('Step %d/%d (t = %.2fs): Arrival time = %.4fs\n', i, max_steps, t_current, arr_val);
    end
end

% Calculate metrics
metrics = struct();
metrics.time_to_target = traj_tau(end) - traj_tau(1);
metrics.control_energy = sum(sum(traj_u.^2));

% Get target value from custom set or BRS
if isfield(options, 'finalSet')
    metrics.final_set_value = eval_u(g, options.finalSet, traj(:,end));
else
    % Use final time slice of data
    metrics.final_set_value = eval_u(g, getData(data, 'end'), traj(:,end));
end

% Interpret metrics differently based on trajectory type
if ~is_frs_trajectory
    % For BRS: We want to reach the target (value <= 0)
    metrics.reached_target = (metrics.final_set_value <= 0);
else
    % For FRS: We want to escape the target (value > 0)
    metrics.escaped_target = (metrics.final_set_value > 0);
    metrics.escape_distance = metrics.final_set_value;  % Value indicates how far we've escaped
end

metrics.final_arrival_time = eval_u(g, arrival_time, traj(:,end));

% Check FRS constraint if applicable
if ~is_frs_trajectory && options.useFRS && isfield(options, 'data_frs')
    metrics = checkFRSConstraint(metrics, traj, traj_tau, options.data_frs, g);
end

end

%% ===== Trajectory computation with gradient precomputation =====
function [traj, traj_tau, traj_u, metrics] = computeTrajectoryWithGradientPrecomputation(g, data, tau, xinit, dynSys, options)
% Optimized trajectory computation method with pre-computation of all gradients

% Determine trajectory type
is_frs_trajectory = strcmp(options.uMode, 'max') || ...
                   (isfield(options, 'reverseFRS') && options.reverseFRS);

% Check if initial state is valid based on trajectory type
if ~is_frs_trajectory
    % For BRS: Check if initial state is in BRS
    initial_value = eval_u(g, getData(data, 'end'), xinit);
    if initial_value > 0
        warning('Initial state is outside the BRS (value = %f)', initial_value);
        % Initialize empty trajectory and metrics
        traj = xinit;
        traj_tau = tau(1);
        traj_u = [];
        
        metrics = struct();
        metrics.time_to_target = 0;
        metrics.final_set_value = initial_value;
        metrics.reached_target = false;
        metrics.control_energy = 0;
        return;
    end
else
    % For FRS: Check if initial state is in or close to target
    if isfield(options, 'finalSet')
        target_value = eval_u(g, options.finalSet, xinit);
        if target_value > 0
            warning(['For FRS trajectories, initial state should ideally be in the target set. ', ...
                     'Current initial state is outside target (value = %.4f)'], target_value);
        end
    end
    
    % Also check if initial state is in FRS
    initial_value = eval_u(g, getData(data, 'end'), xinit);
    if initial_value > 0
        warning(['Initial state may not be in FRS (value = %.4f). ', ...
                 'FRS trajectories work best when starting within the FRS.'], initial_value);
    end
end

% Pre-compute all gradients for each time slice
fprintf('Pre-computing gradients for all time slices...\n');
all_gradients = precomputeGradients(g, data);
fprintf('Gradient pre-computation complete.\n');

% If arrival time is provided, use it for improved guidance
use_arrival_time = isfield(options, 'arrivalTime') && ~isempty(options.arrivalTime);
if use_arrival_time
    fprintf('Computing arrival time gradients for additional guidance...\n');
    arrival_time = options.arrivalTime;
    arrival_grad = computeGradients(g, arrival_time);
    
    % For BRS, we negate the gradient to point toward decreasing values (toward target)
    if ~is_frs_trajectory
        for i = 1:length(arrival_grad)
            arrival_grad{i} = -arrival_grad{i};  % Negate for BRS
        end
    end
    % For FRS, we don't negate the gradient to point toward increasing values (away from target)
end

% Create fast lookup for mapping time to index
tau_to_idx = @(t) min(max(1, find(tau <= t, 1, 'last')), length(tau));

% Initialize trajectory
dt = options.dt;
max_time = options.maxTime;
max_steps = ceil(max_time / dt);

traj = zeros(dynSys.nx, max_steps+1);
traj(:,1) = xinit;
traj_tau = (0:max_steps) * dt;
traj_u = zeros(dynSys.nu, max_steps);

% Initialize dynamic system state
dynSys.x = xinit;

% Set up visualization if enabled
if options.visualize
    if use_arrival_time
        if is_frs_trajectory
            setupVisualization(g, arrival_time, xinit, 'FRS: Gradient+Arrival Time Method');
        else
            setupVisualization(g, arrival_time, xinit, 'BRS: Gradient+Arrival Time Method');
        end
    else
        if is_frs_trajectory
            setupVisualization(g, getData(data, 'end'), xinit, 'FRS: Gradient Method');
        else
            setupVisualization(g, getData(data, 'end'), xinit, 'BRS: Gradient Method');
        end
    end
end

% Main trajectory computation loop
fprintf('Computing trajectory with pre-computed gradients...\n');

for i = 1:max_steps
    % Current state and time
    x_current = traj(:,i);
    t_current = traj_tau(i);
    
    % Choose which gradient to use based on trajectory type and available data
    if use_arrival_time
        % Use arrival time gradient if available
        if ~is_frs_trajectory
            % For BRS: Get arrival time for current state
            arrival_val = eval_u(g, arrival_time, x_current);
            
            % If state is not in BRS (arrival_val is inf), stop computation
            if ~isfinite(arrival_val)
                warning('Current state at step %d is outside the BRS. Stopping computation.', i);
                % Truncate trajectory
                traj = traj(:, 1:i);
                traj_tau = traj_tau(1:i);
                traj_u = traj_u(:, 1:i-1);
                break;
            end
            
            % Use arrival time gradient (already negated for BRS)
            deriv_at_x = eval_u(g, arrival_grad, x_current);
            
            % Visualize if enabled
            if options.visualize
                updateVisualization(traj, i, t_current, arrival_val);
            end
            
            % Check if we've reached the target (very small arrival time)
            if arrival_val <= tau(1) + dt
                fprintf('Target reached at step %d (time %.2fs)!\n', i, t_current);
                % Truncate trajectory
                traj = traj(:, 1:i);
                traj_tau = traj_tau(1:i);
                traj_u = traj_u(:, 1:i-1);
                break;
            end
        else
            % For FRS: Get arrival time for current state
            arrival_val = eval_u(g, arrival_time, x_current);
            
            % If state is not in FRS (arrival_val is inf), stop computation
            if ~isfinite(arrival_val)
                warning('Current state at step %d is outside the FRS. Stopping computation.', i);
                % Truncate trajectory
                traj = traj(:, 1:i);
                traj_tau = traj_tau(1:i);
                traj_u = traj_u(:, 1:i-1);
                break;
            end
            
            % Use arrival time gradient (no need to negate for FRS)
            deriv_at_x = eval_u(g, arrival_grad, x_current);
            
            % Visualize if enabled
            if options.visualize
                updateVisualization(traj, i, t_current, arrival_val);
            end
        end
    else
        % Use precomputed BRS/FRS gradients
        time_idx = tau_to_idx(t_current);
        time_specific_gradients = all_gradients{time_idx};
        deriv_at_x = eval_u(g, time_specific_gradients, x_current);
        
        % Visualize if enabled
        if options.visualize
            updateVisualization(traj, i, t_current);
        end
    end
    
    % Compute optimal control
    u = dynSys.optCtrl(t_current, x_current, deriv_at_x, options.uMode);
    
    % Apply FRS constraint if enabled (for BRS trajectories only)
    if ~is_frs_trajectory && options.useFRS && isfield(options, 'data_frs')
        u = applyFRSConstraint(dynSys, t_current, x_current, deriv_at_x, u, g, options);
    end
    
    % Store control
    traj_u(:,i) = u;
    
    % Check if objective reached based on trajectory type
    if isfield(options, 'finalSet')
        target_value = eval_u(g, options.finalSet, x_current);
        if ~is_frs_trajectory
            % For BRS: Check if target reached
            if target_value <= 0
                fprintf('Target reached at step %d (time %.2fs)!\n', i, t_current);
                % Truncate trajectory
                traj = traj(:, 1:i);
                traj_tau = traj_tau(1:i);
                traj_u = traj_u(:, 1:i-1);
                break;
            end
        else
            % For FRS: Check if sufficiently escaped from target
            if target_value > 0.2  % Use a threshold to ensure we're well outside
                fprintf('Escaped target sufficiently at step %d (time %.2fs)!\n', i, t_current);
                % Truncate trajectory
                traj = traj(:, 1:i);
                traj_tau = traj_tau(1:i);
                traj_u = traj_u(:, 1:i-1);
                break;
            end
        end
    end
    
    % Propagate dynamics for one step
    if ismethod(dynSys, 'optDstb')
        % With disturbance
        if isfield(options, 'dMode')
            d = dynSys.optDstb(t_current, x_current, deriv_at_x, options.dMode);
        else
            d = dynSys.optDstb(t_current, x_current, deriv_at_x);
        end
        dynSys.updateState(u, dt, x_current, d);
    else
        % Without disturbance
        dynSys.updateState(u, dt, x_current);
    end
    
    traj(:,i+1) = dynSys.x;
    
    % Print progress occasionally
    if mod(i, 100) == 0
        fprintf('Step %d/%d (t = %.2fs)\n', i, max_steps, t_current);
    end
end

% Truncate if we didn't use all steps
if i < max_steps
    traj = traj(:, 1:i);
    traj_tau = traj_tau(1:i);
    traj_u = traj_u(:, 1:i-1);
end

% Calculate metrics
metrics = struct();
metrics.time_to_target = traj_tau(end) - traj_tau(1);
metrics.control_energy = sum(sum(traj_u.^2));

% Get target value from custom set or BRS
if isfield(options, 'finalSet')
    metrics.final_set_value = eval_u(g, options.finalSet, traj(:,end));
else
    metrics.final_set_value = eval_u(g, getData(data, 'end'), traj(:,end));
end

% Interpret metrics differently based on trajectory type
if ~is_frs_trajectory
    % For BRS: We want to reach the target (value <= 0)
    metrics.reached_target = (metrics.final_set_value <= 0);
else
    % For FRS: We want to escape the target (value > 0)
    metrics.escaped_target = (metrics.final_set_value > 0);
    metrics.escape_distance = metrics.final_set_value;  % Value indicates how far we've escaped
end

if use_arrival_time
    metrics.final_arrival_time = eval_u(g, arrival_time, traj(:,end));
end

% Check FRS constraint if applicable
if ~is_frs_trajectory && options.useFRS && isfield(options, 'data_frs')
    metrics = checkFRSConstraint(metrics, traj, traj_tau, options.data_frs, g);
end

end

%% ===== Helper functions =====

function all_gradients = precomputeGradients(g, data)
% Precompute gradients for all time slices
    
% Get data dimensions and number of time slices
grid_dims = g.dim;
if ndims(data) == grid_dims + 1
    % Get the last dimension size (time)
    sz = size(data);
    num_time_slices = sz(end);
else
    % Single time slice
    num_time_slices = 1;
end

% Pre-allocate storage for gradients
all_gradients = cell(num_time_slices, 1);

% Compute gradients for each time slice
for t = 1:num_time_slices
    % Get data for this time slice
    data_t = getData(data, t);
    
    % Compute gradient
    all_gradients{t} = computeGradients(g, data_t);
    
    % Show progress
    if mod(t, 10) == 0 || t == num_time_slices
        fprintf('  Computed gradients for time slice %d/%d\n', t, num_time_slices);
    end
end
end

function data_slice = getData(data, idx)
% Extract a slice of data at specified index/time
% Supports 2D, 3D grids and handles special case for 'end'

dims = ndims(data);
grid_dims = dims - 1;  % Assume last dimension is time

% Handle 'end' as a special case
if ischar(idx) && strcmp(idx, 'end')
    % Get data size
    sz = size(data);
    
    % Extract last time slice
    switch grid_dims
        case 2
            data_slice = data(:,:,end);
        case 3
            data_slice = data(:,:,:,end);
        otherwise
            error('Unsupported grid dimension: %d', grid_dims);
    end
else
    % Extract specified time slice
    switch grid_dims
        case 2
            data_slice = data(:,:,idx);
        case 3
            data_slice = data(:,:,:,idx);
        otherwise
            error('Unsupported grid dimension: %d', grid_dims);
    end
end
end

function u_safe = applyFRSConstraint(dynSys, t, x, deriv, u_brs, g, options)
% Apply FRS safety constraints to control input
% This implements a basic blending approach between BRS and FRS control

% First, check if state is in FRS
frs_value = eval_u(g, options.data_frs, x);

% If the state is already outside the FRS, just use the BRS control
if frs_value > 0
    u_safe = u_brs;
    return;
end

% If we're close to the FRS boundary, we need to consider the FRS gradient
if frs_value > -0.1  % Only blend when close to boundary
    % Compute gradient of FRS value function
    deriv_frs = computeGradients(g, options.data_frs);
    deriv_frs_at_x = eval_u(g, deriv_frs, x);
    
    % Get control for staying inside FRS (maximizing FRS value)
    u_frs = dynSys.optCtrl(t, x, deriv_frs_at_x, 'max');
    
    % Check if the controls are in opposite directions
    if any(sign(u_brs) .* sign(u_frs) < 0)  % Check for any conflicting dimensions
        % Controls are conflicting - use weighted average based on FRS value
        % The closer to the FRS boundary, the more we weight the FRS control
        boundary_weight = min(1, max(0, (0.1 - abs(frs_value))/0.1));
        combined_weight = options.frsWeight * boundary_weight;
        
        u_safe = (1 - combined_weight) * u_brs + combined_weight * u_frs;
    else
        % Controls are compatible - use the BRS control
        u_safe = u_brs;
    end
else
    % We're comfortably inside the FRS - use the BRS control
    u_safe = u_brs;
end
end

function metrics = checkFRSConstraint(metrics, traj, traj_tau, data_frs, g)
% Check if trajectory respects FRS constraints and add metrics

metrics.in_frs = true;
metrics.min_frs_value = inf;

for i = 1:size(traj, 2)
    frs_value = eval_u(g, data_frs, traj(:,i));
    metrics.min_frs_value = min(metrics.min_frs_value, frs_value);
    
    if frs_value > 0
        metrics.in_frs = false;
        metrics.frs_violation_time = traj_tau(i);
        break;
    end
end

% If no violations were found
if ~isfield(metrics, 'frs_violation_time')
    metrics.frs_violation_time = inf;
end

return
end

function fig = setupVisualization(g, data, xinit, title_text)
% Set up visualization figure
fig = figure;
set(fig, 'Name', ['Trajectory Computation: ', title_text]);

% If 2D grid
if g.dim == 2
    visSetIm(g, data);
    hold on;
    plot(xinit(1), xinit(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    title(title_text);
    xlabel('x_1');
    ylabel('x_2');
    grid on;
elseif g.dim == 3
    % For 3D, we'll show slices
    % This is a placeholder - actual 3D visualization would be more complex
    visSetIm(g, data);
    hold on;
    plot3(xinit(1), xinit(2), xinit(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    title(title_text);
    xlabel('x_1');
    ylabel('x_2');
    zlabel('x_3');
    grid on;
    view(3);
end
end

function updateVisualization(traj, i, t_current, value)
% Update visualization with current trajectory point
if nargin < 4
    % No value parameter
    plot(traj(1,1:i), traj(2,1:i), 'b-', 'LineWidth', 2);
    plot(traj(1,i), traj(2,i), 'ko', 'MarkerSize', 5, 'LineWidth', 1);
    title(sprintf('Time: %.2f s', t_current));
else
    % With value parameter
    plot(traj(1,1:i), traj(2,1:i), 'b-', 'LineWidth', 2);
    plot(traj(1,i), traj(2,i), 'ko', 'MarkerSize', 5, 'LineWidth', 1);
    title(sprintf('Time: %.2f s, Value: %.4f', t_current, value));
end
drawnow;
end