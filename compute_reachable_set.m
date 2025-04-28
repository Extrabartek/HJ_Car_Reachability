function result_folder = compute_reachable_set(velocities, control_limits, varargin)
% COMPUTE_REACHABLE_SET Computes reachable sets for vehicle with different control limits
%
% This consolidated function replaces:
%   - compute_brs_mzmax.m
%   - compute_frs_mzmax.m
%   - compute_brs_dvmax.m
%   - compute_frs_dvmax.m
%
% Inputs:
%   velocities     - Array of longitudinal velocities to test [m/s]
%   control_limits - Array of control limits to test:
%                    - If controlType = 'mz': max yaw moment [N·m]
%                    - If controlType = 'dv': max steering rate [rad/s]
%   varargin       - Optional parameter-value pairs:
%                    - 'direction'  - Computation direction: 'backward' or 'forward'
%                                     (default: 'backward')
%                    - 'controlType'- Control type: 'mz' (yaw moment) or 'dv' (steering rate)
%                                     (default: 'mz')
%                    - 'tMax'       - Maximum simulation time [s] (default: 1.0)
%                    - 'dt'         - Time step [s] (default: 0.05)
%                    - 'gridSize'   - Grid size array - length depends on controlType:
%                                     - 'mz': [nx, ny] for [yaw rate, sideslip]
%                                     - 'dv': [nx, ny, nz] for [yaw rate, sideslip, steering]
%                                     (default: [71, 71] or [51, 51, 51])
%                    - 'gridMin'    - Grid minimum values [rad] - length depends on controlType
%                                     (default: depends on controlType)
%                    - 'gridMax'    - Grid maximum values [rad] - length depends on controlType
%                                     (default: depends on controlType)
%                    - 'targetSize' - Size of target set [rad] - length depends on controlType
%                                     (default: depends on controlType)
%                    - 'uMode'      - Control mode:
%                                     - For 'backward': usually 'min' for target reaching
%                                     - For 'forward': usually 'max' for reachability
%                                     (default: depends on direction)
%                    - 'accuracy'   - Accuracy level: 'low', 'medium', 'high', 'veryHigh'
%                                     (default: 'high')
%                    - 'visualize'  - Whether to show visualization (default: true)
%
% Output:
%   result_folder  - Path to the folder where results are saved
%
% Examples:
%   % Compute BRS with yaw moment control:
%   folder = compute_reachable_set([30], [10000], 'direction', 'backward', 'controlType', 'mz');
%
%   % Compute FRS with steering rate control:
%   folder = compute_reachable_set([15, 30], [deg2rad(10), deg2rad(20)], ...
%                                 'direction', 'forward', 'controlType', 'dv', 'tMax', 0.5);

%% Parse inputs
p = inputParser;
p.addRequired('velocities', @isnumeric);
p.addRequired('control_limits', @isnumeric);
p.addParameter('direction', 'backward', @(x) ismember(lower(x), {'backward', 'forward'}));
p.addParameter('controlType', 'mz', @(x) ismember(lower(x), {'mz', 'dv'}));
p.addParameter('tMax', 1.0, @isnumeric);
p.addParameter('dt', 0.05, @isnumeric);
p.addParameter('gridSize', [], @isnumeric);
p.addParameter('gridMin', [], @isnumeric);
p.addParameter('gridMax', [], @isnumeric);
p.addParameter('targetSize', [], @isnumeric);
p.addParameter('uMode', '', @ischar);
p.addParameter('accuracy', 'high', @(x) ismember(lower(x), {'low', 'medium', 'high', 'veryhigh'}));
p.addParameter('visualize', true, @islogical);

p.parse(velocities, control_limits, varargin{:});
opts = p.Results;

% Convert inputs to lowercase for consistency
opts.direction = lower(opts.direction);
opts.controlType = lower(opts.controlType);
opts.accuracy = lower(opts.accuracy);

% Define default grid parameters based on control type
if strcmp(opts.controlType, 'mz')
    % Defaults for yaw moment control (2D state space)
    if isempty(opts.gridSize)
        opts.gridSize = [71, 71];
    end
    if isempty(opts.gridMin)
        opts.gridMin = [deg2rad(-150), deg2rad(-25)];
    end
    if isempty(opts.gridMax)
        opts.gridMax = [deg2rad(150), deg2rad(25)];
    end
    if isempty(opts.targetSize)
        opts.targetSize = [deg2rad(15), deg2rad(6)];
    end
else
    % Defaults for steering rate control (3D state space)
    if isempty(opts.gridSize)
        opts.gridSize = [51, 51, 51];
    end
    if isempty(opts.gridMin)
        opts.gridMin = [deg2rad(-150), deg2rad(-25), deg2rad(-10)];
    end
    if isempty(opts.gridMax)
        opts.gridMax = [deg2rad(150), deg2rad(25), deg2rad(10)];
    end
    if isempty(opts.targetSize)
        opts.targetSize = [deg2rad(15), deg2rad(3), deg2rad(1)];
    end
end

% Set default control mode based on direction if not specified
if isempty(opts.uMode)
    if strcmp(opts.direction, 'backward')
        opts.uMode = 'min'; % For target reaching in BRS
    else
        opts.uMode = 'max'; % For maximizing reachable area in FRS
    end
end

%% Create results directory with timestamp
% Create parent results directory if it doesn't exist
parent_results_dir = fullfile(pwd, 'results');
if ~exist(parent_results_dir, 'dir')
    mkdir(parent_results_dir);
    
    % Create a .gitignore file in the parent results directory
    gitignore_path = fullfile(parent_results_dir, '.gitignore');
    gitignore_content = ['# Ignore all files in this directory\n', ...
                         '*\n', ...
                         '# Except the .gitignore itself\n', ...
                         '!.gitignore\n'];
    fid = fopen(gitignore_path, 'w');
    if fid ~= -1
        fprintf(fid, '%s', gitignore_content);
        fclose(fid);
        disp('Created .gitignore file in results directory.');
    else
        warning('Could not create .gitignore file.');
    end
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');

% Create folder name based on computation type
if strcmp(opts.direction, 'backward')
    type_prefix = 'brs';
else
    type_prefix = 'frs';
end

if strcmp(opts.controlType, 'mz')
    control_name = 'mz';
    control_value_min = min(control_limits);
    control_value_max = max(control_limits);
else
    control_name = 'dvmax';
    % Convert to degrees for folder name
    control_value_min = round(min(control_limits) * 180/pi);
    control_value_max = round(max(control_limits) * 180/pi);
end

% Add steering model indicator to folder name if using steering control
if strcmp(opts.controlType, 'dv')
    folder_name = sprintf('steered_%s_results_%s', type_prefix, timestamp);
else
    folder_name = sprintf('%s_results_%s', type_prefix, timestamp);
end

% Create descriptive folder name with parameters
param_desc = sprintf('_vx%d-%d_%s%d-%d', ...
    min(velocities), max(velocities), ...
    control_name, control_value_min, control_value_max);
    
result_folder = fullfile(parent_results_dir, [folder_name, param_desc]);

% Create the directory if it doesn't exist
if ~exist(result_folder, 'dir')
    mkdir(result_folder);
end

disp(['Creating results directory: ', result_folder]);

%% Setup time vector
t0 = 0;
tMax = opts.tMax;
dt = opts.dt;
tau = t0:dt:tMax;

%% Create grid
disp('Creating grid...');
grid_min = opts.gridMin;
grid_max = opts.gridMax;
N = opts.gridSize;
pdDims = []; % No periodic dimensions
g = createGrid(grid_min, grid_max, N, pdDims);

%% Create target set
disp('Creating target set...');
target_size = opts.targetSize;
data0 = shapeRectangleByCenter(g, zeros(length(target_size), 1), target_size);

%% Setup base parameters based on control type
if strcmp(opts.controlType, 'mz')
    % Base parameters for yaw moment control: [m, Vx, Lf, Lr, Iz, mu, Mzmax, Mzmin, Cf, Cr]
    base_params = [1430; 50; 1.05; 1.61; 2059.2; 1; 10000; -10000; 90700; 109000];
    control_param_idx = [7, 8]; % Indices of Mzmax and Mzmin parameters
else
    % Base parameters for steering rate control: [m, Vx, Lf, Lr, Iz, mu, dv_max, Cf, Cr]
    base_params = [1430; 50; 1.05; 1.61; 2059.2; 1; deg2rad(30); 90700; 109000];
    control_param_idx = 7; % Index of dv_max parameter
end

%% Store simulation parameters
sim_params = struct();
sim_params.velocities = velocities;
if strcmp(opts.controlType, 'mz')
    sim_params.mzmax_values = control_limits;
else
    sim_params.dvmax_values = control_limits;
end
sim_params.grid_min = grid_min;
sim_params.grid_max = grid_max;
sim_params.grid_size = N;
sim_params.tau = tau;
sim_params.base_params = base_params;
sim_params.target_size = target_size;
sim_params.uMode = opts.uMode;
sim_params.direction = opts.direction;
sim_params.controlType = opts.controlType;
sim_params.timestamp = timestamp;

% Save simulation parameters
save(fullfile(result_folder, 'sim_params.mat'), 'sim_params');

%% Initialize storage structures
all_data = cell(length(velocities), length(control_limits));
all_data_full = cell(length(velocities), length(control_limits));
control_data = cell(length(velocities), length(control_limits));

%% Loop through each velocity and control limit combination
for v_idx = 1:length(velocities)
    % Update the velocity parameter
    params = base_params;
    params(2) = velocities(v_idx);
    
    for c_idx = 1:length(control_limits)
        % Update control limit parameters
        if strcmp(opts.controlType, 'mz')
            % For yaw moment control, set Mzmax and Mzmin
            params(control_param_idx(1)) = control_limits(c_idx);      % Mzmax
            params(control_param_idx(2)) = -control_limits(c_idx);     % Mzmin
            
            % Display progress
            fprintf('Computing %s for velocity = %d m/s, Mzmax = %d N·m...\n', ...
                    upper(type_prefix), velocities(v_idx), control_limits(c_idx));
            
            % Create vehicle model
            dCar = NonlinearBicycle([0; 0], params);
        else
            % For steering rate control, set dv_max
            params(control_param_idx) = control_limits(c_idx);         % dv_max
            
            % Display progress
            fprintf('Computing %s for velocity = %d m/s, dv_max = %.2f rad/s (%.2f deg/s)...\n', ...
                    upper(type_prefix), velocities(v_idx), control_limits(c_idx), control_limits(c_idx)*180/pi);
            
            % Create vehicle model
            dCar = NonlinearBicycleSteered(zeros(3, 1), params);
        end
        
        % Put grid and dynamic systems into schemeData
        schemeData.grid = g;
        schemeData.dynSys = dCar;
        schemeData.accuracy = opts.accuracy;
        schemeData.uMode = opts.uMode;
        
        % Set computation direction (forward/backward)
        if strcmp(opts.direction, 'forward')
            schemeData.tMode = 'forward';
        end
        
        % Setup visualization options
        HJIextraArgs.visualize = opts.visualize;
        if opts.visualize
            HJIextraArgs.visualize.valueSet = true;
            HJIextraArgs.visualize.initialValueSet = true;
            HJIextraArgs.visualize.figNum = 1;
            HJIextraArgs.visualize.deleteLastPlot = true;
        end
        
        % Solve HJ PDE to get reachable sets
        disp('Solving Hamilton-Jacobi PDE...');
        tic;
        [data, tau2, extraOuts] = HJIPDE_solve(data0, tau, schemeData, 'none', HJIextraArgs);
        computation_time = toc;
        fprintf('Computation completed in %.2f seconds.\n', computation_time);
        
        % Store the computed data
        if strcmp(opts.controlType, 'mz')
            % 2D state space
            all_data{v_idx, c_idx} = data(:,:,end);
            all_data_full{v_idx, c_idx} = data;
        else
            % 3D state space
            all_data{v_idx, c_idx} = data(:,:,:,end);
            all_data_full{v_idx, c_idx} = data;
        end
        
        % Calculate optimal control
        disp('Computing optimal control strategy...');
        
        % Extract state dimensions based on control type
        state_dims = length(opts.gridSize);
        
        % Get gradient of value function
        if strcmp(opts.controlType, 'mz')
            % For 2D state space
            derivs = computeGradients(g, data(:,:,end));
            
            % Initialize control grid
            control_grid = zeros(size(g.xs{1}));
            
            % Determine optimal control based on the sign of the derivative
            if strcmp(opts.uMode, 'min')
                % For target reaching
                control_grid(derivs{2} >= 0) = -control_limits(c_idx);  % Mzmin
                control_grid(derivs{2} < 0) = control_limits(c_idx);    % Mzmax
            else
                % For target avoidance
                control_grid(derivs{2} >= 0) = control_limits(c_idx);   % Mzmax
                control_grid(derivs{2} < 0) = -control_limits(c_idx);   % Mzmin
            end
        else
            % For 3D state space
            derivs = computeGradients3D(g, data(:,:,:,end));
            
            % Initialize control grid
            control_grid = zeros(size(g.xs{1}));
            
            % Determine optimal control based on the sign of the derivative
            if strcmp(opts.uMode, 'min')
                % For target reaching
                control_grid(derivs{3} >= 0) = -control_limits(c_idx);  % -dv_max
                control_grid(derivs{3} < 0) = control_limits(c_idx);    % dv_max
            else
                % For target avoidance
                control_grid(derivs{3} >= 0) = control_limits(c_idx);   % dv_max
                control_grid(derivs{3} < 0) = -control_limits(c_idx);   % -dv_max
            end
        end
        
        % Store the control grid
        control_data{v_idx, c_idx} = control_grid;
        
        % Save individual result file for each combination
        if strcmp(opts.controlType, 'mz')
            result_filename = sprintf('%s_v%d_mz%d.mat', type_prefix, velocities(v_idx), control_limits(c_idx));
        else
            result_filename = sprintf('%s_v%d_dvmax%.0f.mat', type_prefix, velocities(v_idx), control_limits(c_idx)*180/pi);
        end
        
        save(fullfile(result_folder, result_filename), ...
             'g', 'data0', 'data', 'control_grid', 'params', 'tau', 'computation_time');
        
        fprintf('Saved result to %s\n', fullfile(result_folder, result_filename));
    end
end

% Save combined results
disp('Saving combined results...');
if strcmp(opts.controlType, 'mz')
    save(fullfile(result_folder, [type_prefix, '_combined_results.mat']), ...
         'g', 'data0', 'all_data', 'all_data_full', 'control_data', ...
         'velocities', 'control_limits', 'tau', 'base_params');
else
    % Rename control_limits to dvmax_values for backward compatibility
    dvmax_values = control_limits;
    save(fullfile(result_folder, [type_prefix, '_combined_results.mat']), ...
         'g', 'data0', 'all_data', 'all_data_full', 'control_data', ...
         'velocities', 'dvmax_values', 'tau', 'base_params');
end

disp('Computation complete!');
disp(['Results saved to: ', result_folder]);

end

% Helper function to compute gradients for 2D value function
function derivs = computeGradients(g, data)
    % Initialize derivatives
    derivs = cell(g.dim, 1);
    
    % Use central differences for robustness
    for i = 1:g.dim
        % Initialize arrays for the derivative
        deriv = zeros(size(data));
        
        % Calculate derivatives with appropriate boundary handling
        if i == 1
            % For the first dimension
            deriv(2:end-1, :) = (data(3:end, :) - data(1:end-2, :)) / (2*g.dx(i));
            % Handle boundaries with one-sided differences
            deriv(1, :) = (data(2, :) - data(1, :)) / g.dx(i);
            deriv(end, :) = (data(end, :) - data(end-1, :)) / g.dx(i);
        else
            % For the second dimension
            deriv(:, 2:end-1) = (data(:, 3:end) - data(:, 1:end-2)) / (2*g.dx(i));
            % Handle boundaries with one-sided differences
            deriv(:, 1) = (data(:, 2) - data(:, 1)) / g.dx(i);
            deriv(:, end) = (data(:, end) - data(:, end-1)) / g.dx(i);
        end
        
        derivs{i} = deriv;
    end
end

% Helper function to compute gradients for 3D value function
function derivs = computeGradients3D(g, data)
    % Initialize derivatives
    derivs = cell(g.dim, 1);
    
    % Use central differences for robustness
    for i = 1:g.dim
        % Initialize arrays for the derivative
        deriv = zeros(size(data));
        
        % Calculate derivatives using central differences
        % For a 3D grid, need to handle derivatives along each dimension
        switch i
            case 1 % First dimension
                % Interior points
                deriv(2:end-1,:,:) = (data(3:end,:,:) - data(1:end-2,:,:)) / (2*g.dx(i));
                % Boundary points
                deriv(1,:,:) = (data(2,:,:) - data(1,:,:)) / g.dx(i);
                deriv(end,:,:) = (data(end,:,:) - data(end-1,:,:)) / g.dx(i);
                
            case 2 % Second dimension
                % Interior points
                deriv(:,2:end-1,:) = (data(:,3:end,:) - data(:,1:end-2,:)) / (2*g.dx(i));
                % Boundary points
                deriv(:,1,:) = (data(:,2,:) - data(:,1,:)) / g.dx(i);
                deriv(:,end,:) = (data(:,end,:) - data(:,end-1,:)) / g.dx(i);
                
            case 3 % Third dimension
                % Interior points
                deriv(:,:,2:end-1) = (data(:,:,3:end) - data(:,:,1:end-2)) / (2*g.dx(i));
                % Boundary points
                deriv(:,:,1) = (data(:,:,2) - data(:,:,1)) / g.dx(i);
                deriv(:,:,end) = (data(:,:,end) - data(:,:,end-1)) / g.dx(i);
        end
        
        derivs{i} = deriv;
    end
end