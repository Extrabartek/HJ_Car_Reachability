function result_folder = compute_reachable_set(velocities, control_limits, varargin)
% COMPUTE_REACHABLE_SET Computes reachable sets for vehicle with different control limits
%
% This consolidated function handles computation for multiple model types:
%   - Bicycle models (standard and steered)
%   - Double Integrator
%   - Dubins Car
%
% Inputs:
%   velocities     - Array of longitudinal velocities to test [m/s]
%                    (for bicycle models)
%                    OR empty array [] for other models
%   control_limits - Array of control limits to test:
%                    - If controlType = 'mz': max yaw moment [N·m]
%                    - If controlType = 'dv': max steering rate [rad/s]
%                    - If modelType = 'doubleInt': acceleration limits [m/s²]
%                    - If modelType = 'dubinsCar': turning rate limits [rad/s]
%   varargin       - Optional parameter-value pairs:
%                    - 'direction'  - Computation direction: 'backward' or 'forward'
%                                     (default: 'backward')
%                    - 'controlType'- Control type: 'mz' (yaw moment) or 'dv' (steering rate)
%                                     (default: 'mz')
%                    - 'modelType'  - Model type: 'bicycle', 'doubleInt', 'dubinsCar'
%                                     (default: 'bicycle')
%                    - 'tMax'       - Maximum simulation time [s] (default: 1.0)
%                    - 'dt'         - Time step [s] (default: 0.05)
%                    - 'gridSize'   - Grid size array - depends on model type
%                    - 'gridMin'    - Grid minimum values - depends on model type
%                    - 'gridMax'    - Grid maximum values - depends on model type
%                    - 'targetSize' - Size of target set - depends on model type
%                    - 'uMode'      - Control mode
%                    - 'accuracy'   - Accuracy level: 'low', 'medium', 'high', 'veryHigh'
%                                     (default: 'high')
%                    - 'visualize'  - Whether to show visualization (default: true)
%                    - Model-specific parameters:
%                       - 'speed'      - For Dubins Car: constant speed
%                       - 'dRange'     - For Dubins Car and Double Integrator: disturbance range
%                       - 'dims'       - For Dubins Car and Double Integrator: active dimensions
%
% Output:
%   result_folder  - Path to the folder where results are saved

%% Parse inputs
p = inputParser;
p.addRequired('velocities', @isnumeric);
p.addRequired('control_limits', @isnumeric);
p.addParameter('direction', 'backward', @(x) ismember(lower(x), {'backward', 'forward'}));
p.addParameter('controlType', 'mz', @(x) ismember(lower(x), {'mz', 'dv'}));
p.addParameter('modelType', 'bicycle', @(x) ismember(lower(x), {'bicycle', 'doubleint', 'dubinscar'}));
p.addParameter('tMax', 1.0, @isnumeric);
p.addParameter('dt', 0.05, @isnumeric);
p.addParameter('gridSize', [], @isnumeric);
p.addParameter('gridMin', [], @isnumeric);
p.addParameter('gridMax', [], @isnumeric);
p.addParameter('targetSize', [], @isnumeric);
p.addParameter('uMode', '', @ischar);
p.addParameter('accuracy', 'high', @(x) ismember(lower(x), {'low', 'medium', 'high', 'veryhigh'}));
p.addParameter('visualize', true, @islogical);

% Model-specific parameters
p.addParameter('speed', 5, @isnumeric);           % For Dubins Car: constant speed
p.addParameter('dRange', {[0;0;0], [0;0;0]}, @(x) isnumeric(x) || iscell(x)); % Disturbance range
p.addParameter('dims', [], @isnumeric);           % Active dimensions

p.parse(velocities, control_limits, varargin{:});
opts = p.Results;

% Convert inputs to lowercase for consistency
opts.direction = lower(opts.direction);
opts.controlType = lower(opts.controlType);
opts.modelType = lower(opts.modelType);
opts.accuracy = lower(opts.accuracy);

%% Set default parameters based on model type
if strcmp(opts.modelType, 'bicycle')
    % Defaults for bicycle model
    if strcmp(opts.controlType, 'mz')
        % 2D Bicycle with yaw moment control
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
        % 3D Steered bicycle model
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
elseif strcmp(opts.modelType, 'doubleint')
    % Defaults for Double Integrator
    if isempty(opts.gridSize)
        opts.gridSize = [101, 101];
    end
    if isempty(opts.gridMin)
        opts.gridMin = [-5, -5];
    end
    if isempty(opts.gridMax)
        opts.gridMax = [5, 5];
    end
    if isempty(opts.targetSize)
        opts.targetSize = [0.5];
    end
    if isempty(opts.dims)
        opts.dims = 1:2;
    end
elseif strcmp(opts.modelType, 'dubinscar')
    % Defaults for Dubins Car
    if isempty(opts.gridSize)
        opts.gridSize = [51, 51, 51];
    end
    if isempty(opts.gridMin)
        opts.gridMin = [-5, -5, -pi];
    end
    if isempty(opts.gridMax)
        opts.gridMax = [5, 5, pi];
    end
    if isempty(opts.targetSize)
        opts.targetSize = [1.0];
    end
    if isempty(opts.dims)
        opts.dims = 1:3;
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

% Create folder name based on computation type and model
if strcmp(opts.direction, 'backward')
    type_prefix = 'brs';
else
    type_prefix = 'frs';
end

folder_name = sprintf('%s_%s_results_%s', opts.modelType, type_prefix, timestamp);

% Add parameter description to folder name based on model type
if strcmp(opts.modelType, 'bicycle')
    % For bicycle models, include velocity and control info
    if strcmp(opts.controlType, 'mz')
        control_name = 'mz';
        vmin = min(velocities);
        vmax = max(velocities);
        control_min = min(control_limits);
        control_max = max(control_limits);
    else
        control_name = 'dvmax';
        vmin = min(velocities);
        vmax = max(velocities);
        % Convert to degrees for folder name
        control_min = round(min(control_limits) * 180/pi);
        control_max = round(max(control_limits) * 180/pi);
    end
    param_desc = sprintf('_vx%d-%d_%s%d-%d', vmin, vmax, control_name, control_min, control_max);
elseif strcmp(opts.modelType, 'doubleint')
    % For Double Integrator, include control limits
    control_min = min(control_limits);
    control_max = max(control_limits);
    param_desc = sprintf('_acc%d-%d', control_min, control_max);
elseif strcmp(opts.modelType, 'dubinscar')
    % For Dubins Car, include speed and turning rate
    speed = opts.speed;
    turn_min = round(min(control_limits) * 180/pi);
    turn_max = round(max(control_limits) * 180/pi);
    param_desc = sprintf('_v%d_turn%d-%d', speed, turn_min, turn_max);
end
    
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

%% Create grid based on model type
disp('Creating grid...');
if strcmp(opts.modelType, 'bicycle')
    if strcmp(opts.controlType, 'dv')
        % 3D grid for steered bicycle
        pdDims = []; % No periodic dimensions
        g = createGrid(opts.gridMin, opts.gridMax, opts.gridSize, pdDims);
    else
        % 2D grid for standard bicycle
        pdDims = []; % No periodic dimensions
        g = createGrid(opts.gridMin, opts.gridMax, opts.gridSize, pdDims);
    end
elseif strcmp(opts.modelType, 'doubleint')
    % 2D grid for Double Integrator
    g = createGrid(opts.gridMin, opts.gridMax, opts.gridSize);
elseif strcmp(opts.modelType, 'dubinscar')
    % 3D grid for Dubins Car with periodic heading
    pdDims = 3; % 3rd dimension (heading) is periodic
    g = createGrid(opts.gridMin, opts.gridMax, opts.gridSize, pdDims);
end

%% Create target set based on model type
disp('Creating target set...');
if strcmp(opts.modelType, 'bicycle')
    if strcmp(opts.controlType, 'dv')
        % 3D target for steered bicycle
        data0 = shapeRectangleByCenter(g, zeros(3, 1), opts.targetSize);
    else
        % 2D target for standard bicycle
        data0 = shapeRectangleByCenter(g, zeros(2, 1), opts.targetSize);
    end
elseif strcmp(opts.modelType, 'doubleint')
    % Target around origin for Double Integrator
    data0 = shapeCylinder(g, [], [0; 0], opts.targetSize(1));
elseif strcmp(opts.modelType, 'dubinscar')
    % Target at specific position for Dubins Car
    data0 = shapeCylinder(g, 3, [0; 0; 0], opts.targetSize(1));
end

%% Setup base parameters and create dynamic system based on model type
if strcmp(opts.modelType, 'bicycle')
    if strcmp(opts.controlType, 'mz')
        % Base parameters for yaw moment control: [m, Vx, Lf, Lr, Iz, mu, Mzmax, Mzmin, Cf, Cr]
        base_params = [1430; 50; 1.05; 1.61; 2059.2; 1; 10000; -10000; 90700; 109000];
        control_param_idx = [7, 8]; % Indices of Mzmax and Mzmin parameters
    else
        % Base parameters for steering rate control: [m, Vx, Lf, Lr, Iz, mu, dv_max, Cf, Cr]
        base_params = [1430; 50; 1.05; 1.61; 2059.2; 1; deg2rad(30); 90700; 109000];
        control_param_idx = 7; % Index of dv_max parameter
    end
elseif strcmp(opts.modelType, 'doubleint')
    % For Double Integrator, store control limits
    base_params = control_limits; % Store control limits directly
elseif strcmp(opts.modelType, 'dubinscar')
    % For Dubins Car, store speed and control limits
    base_params = [opts.speed; control_limits(1)]; % [speed; wRange]
end

%% Store simulation parameters
sim_params = struct();
if strcmp(opts.modelType, 'bicycle')
    sim_params.velocities = velocities;
    if strcmp(opts.controlType, 'mz')
        sim_params.mzmax_values = control_limits;
    else
        sim_params.dvmax_values = control_limits;
    end
elseif strcmp(opts.modelType, 'doubleint')
    sim_params.acceleration_limits = control_limits;
    sim_params.dimensions = opts.dims;
    sim_params.disturbance_range = opts.dRange;
elseif strcmp(opts.modelType, 'dubinscar')
    sim_params.speed = opts.speed;
    sim_params.turning_limits = control_limits;
    sim_params.dimensions = opts.dims;
    sim_params.disturbance_range = opts.dRange;
end

sim_params.grid_min = opts.gridMin;
sim_params.grid_max = opts.gridMax;
sim_params.grid_size = opts.gridSize;
sim_params.tau = tau;
sim_params.base_params = base_params;
sim_params.target_size = opts.targetSize;
sim_params.uMode = opts.uMode;
sim_params.direction = opts.direction;
sim_params.controlType = opts.controlType;
sim_params.modelType = opts.modelType;
sim_params.timestamp = timestamp;

% Save simulation parameters
save(fullfile(result_folder, 'sim_params.mat'), 'sim_params');

%% Initialize storage structures
all_data = cell(length(velocities), length(control_limits));
all_data_full = cell(length(velocities), length(control_limits));
control_data = cell(length(velocities), length(control_limits));

%% Loop through each parameter combination
% For bicycle: iterate through velocities and control limits
% For other models: just iterate through control limits (with single dummy velocity)
if strcmp(opts.modelType, 'bicycle')
    velocity_range = velocities;
else
    velocity_range = 1; % Dummy velocity for non-bicycle models
end

for v_idx = 1:length(velocity_range)
    % Set parameters based on model type
    if strcmp(opts.modelType, 'bicycle')
        % Update the velocity parameter for bicycle models
        params = base_params;
        params(2) = velocities(v_idx);
    end
    
    for c_idx = 1:length(control_limits)
        % Set control parameters based on model type
        if strcmp(opts.modelType, 'bicycle')
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
        elseif strcmp(opts.modelType, 'doubleint')
            % For Double Integrator
            % Set control range
            urange = [-control_limits(c_idx), control_limits(c_idx)];
            
            % Display progress
            fprintf('Computing %s for Double Integrator with acceleration limits = [%g, %g]...\n', ...
                    upper(type_prefix), -control_limits(c_idx), control_limits(c_idx));
            
            % Create Double Integrator model with specified dimensions and disturbance
            dCar = DoubleInt(zeros(2, 1), urange, opts.dRange, opts.dims);
        elseif strcmp(opts.modelType, 'dubinscar')
            % For Dubins Car
            % Set turning rate limits
            wRange = [-control_limits(c_idx), control_limits(c_idx)];
            
            % Display progress
            fprintf('Computing %s for Dubins Car with speed = %g, turning limits = [%g, %g] rad/s...\n', ...
                    upper(type_prefix), opts.speed, -control_limits(c_idx), control_limits(c_idx));
            
            % Create Dubins Car model with specified speed, turning limits, dimensions, and disturbance
            dCar = DubinsCar([0; 0; 0], wRange, opts.speed, opts.dRange, opts.dims);
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
        if opts.visualize
            HJIextraArgs.visualize.valueSet = true;
            HJIextraArgs.visualize.initialValueSet = true;
            HJIextraArgs.visualize.figNum = 1;
            HJIextraArgs.visualize.deleteLastPlot = true;
        else
            HJIextraArgs.visualize = false;
        end
        
        % Solve HJ PDE to get reachable sets
        disp('Solving Hamilton-Jacobi PDE...');
        tic;
        [data, tau2, extraOuts] = HJIPDE_solve(data0, tau, schemeData, 'none', HJIextraArgs);
        computation_time = toc;
        fprintf('Computation completed in %.2f seconds.\n', computation_time);
        
        % Store the computed data based on model dimensions
        if (strcmp(opts.modelType, 'bicycle') && strcmp(opts.controlType, 'mz')) || ...
           strcmp(opts.modelType, 'doubleint')
            % 2D state space
            all_data{v_idx, c_idx} = data(:,:,end);
            all_data_full{v_idx, c_idx} = data;
        else
            % 3D state space
            all_data{v_idx, c_idx} = data(:,:,:,end);
            all_data_full{v_idx, c_idx} = data;
        end
        
        % Calculate optimal control - this may need model-specific implementations
        disp('Computing optimal control strategy...');
        
        % Extract state dimensions based on model and control type
        if (strcmp(opts.modelType, 'bicycle') && strcmp(opts.controlType, 'mz')) || ...
           strcmp(opts.modelType, 'doubleint')
            % 2D state space
            state_dims = 2;
            
            % Compute gradients
            if strcmp(opts.modelType, 'bicycle')
                derivs = computeGradients(g, data(:,:,end));
            else
                derivs = computeGradients(g, data(:,:,end));
            end
            
            % Initialize control grid
            control_grid = zeros(size(g.xs{1}));
            
            % Determine optimal control based on the model type
            if strcmp(opts.modelType, 'bicycle')
                % For bicycle with yaw moment
                if strcmp(opts.uMode, 'min')
                    % For target reaching
                    control_grid(derivs{2} >= 0) = -control_limits(c_idx);  % Mzmin
                    control_grid(derivs{2} < 0) = control_limits(c_idx);    % Mzmax
                else
                    % For target avoidance
                    control_grid(derivs{2} >= 0) = control_limits(c_idx);   % Mzmax
                    control_grid(derivs{2} < 0) = -control_limits(c_idx);   % Mzmin
                end
            elseif strcmp(opts.modelType, 'doubleint')
                % For Double Integrator
                if strcmp(opts.uMode, 'min')
                    % For target reaching
                    control_grid(derivs{2} >= 0) = -control_limits(c_idx);  % Minimum acceleration
                    control_grid(derivs{2} < 0) = control_limits(c_idx);    % Maximum acceleration
                else
                    % For target avoidance
                    control_grid(derivs{2} >= 0) = control_limits(c_idx);   % Maximum acceleration
                    control_grid(derivs{2} < 0) = -control_limits(c_idx);   % Minimum acceleration
                end
            end
        else
            % 3D state space (steered bicycle or Dubins Car)
            state_dims = 3;
            
            % Compute gradients
            if strcmp(opts.modelType, 'bicycle')
                derivs = computeGradients3D(g, data(:,:,:,end));
            else
                derivs = computeGradients3D(g, data(:,:,:,end));
            end
            
            % Initialize control grid
            control_grid = zeros(size(g.xs{1}));
            
            % Determine optimal control based on the model type
            if strcmp(opts.modelType, 'bicycle')
                % For steered bicycle
                if strcmp(opts.uMode, 'min')
                    % For target reaching
                    control_grid(derivs{3} >= 0) = -control_limits(c_idx);  % -dv_max
                    control_grid(derivs{3} < 0) = control_limits(c_idx);    % dv_max
                else
                    % For target avoidance
                    control_grid(derivs{3} >= 0) = control_limits(c_idx);   % dv_max
                    control_grid(derivs{3} < 0) = -control_limits(c_idx);   % -dv_max
                end
            elseif strcmp(opts.modelType, 'dubinscar')
                % For Dubins Car
                if strcmp(opts.uMode, 'min')
                    % For target reaching
                    control_grid(derivs{3} >= 0) = -control_limits(c_idx);  % Minimum turning rate
                    control_grid(derivs{3} < 0) = control_limits(c_idx);    % Maximum turning rate
                else
                    % For target avoidance
                    control_grid(derivs{3} >= 0) = control_limits(c_idx);   % Maximum turning rate
                    control_grid(derivs{3} < 0) = -control_limits(c_idx);   % Minimum turning rate
                end
            end
        end
        
        % Store the control grid
        control_data{v_idx, c_idx} = control_grid;
        
        % Save individual result file for each combination
        if strcmp(opts.modelType, 'bicycle')
            if strcmp(opts.controlType, 'mz')
                result_filename = sprintf('%s_v%d_mz%d.mat', type_prefix, velocities(v_idx), control_limits(c_idx));
            else
                result_filename = sprintf('%s_v%d_dvmax%.0f.mat', type_prefix, velocities(v_idx), control_limits(c_idx)*180/pi);
            end
        elseif strcmp(opts.modelType, 'doubleint')
            result_filename = sprintf('%s_doubleint_acc%g.mat', type_prefix, control_limits(c_idx));
        elseif strcmp(opts.modelType, 'dubinscar')
            result_filename = sprintf('%s_dubinscar_v%g_turn%.0f.mat', type_prefix, opts.speed, control_limits(c_idx)*180/pi);
        end
        
        % Create a structure with all relevant data for this computation
        result_data = struct();
        result_data.g = g;
        result_data.data0 = data0;
        result_data.data = data;
        result_data.control_grid = control_grid;
        result_data.params = base_params;
        result_data.tau = tau;
        result_data.computation_time = computation_time;
        
        % Add model-specific parameters
        if strcmp(opts.modelType, 'doubleint')
            result_data.urange = [-control_limits(c_idx), control_limits(c_idx)];
            result_data.drange = opts.dRange;
            result_data.dims = opts.dims;
        elseif strcmp(opts.modelType, 'dubinscar')
            result_data.speed = opts.speed;
            result_data.wRange = [-control_limits(c_idx), control_limits(c_idx)];
            result_data.dRange = opts.dRange;
            result_data.dims = opts.dims;
        end
        
        save(fullfile(result_folder, result_filename), '-struct', 'result_data');
        
        fprintf('Saved result to %s\n', fullfile(result_folder, result_filename));
    end
end

% Save combined results
disp('Saving combined results...');
combined_results = struct();
combined_results.g = g;
combined_results.data0 = data0;
combined_results.all_data = all_data;
combined_results.all_data_full = all_data_full;
combined_results.control_data = control_data;
combined_results.tau = tau;
combined_results.base_params = base_params;

% Add model-specific parameters to combined results
if strcmp(opts.modelType, 'bicycle')
    combined_results.velocities = velocities;
    if strcmp(opts.controlType, 'mz')
        combined_results.mzmax_values = control_limits;
    else
        combined_results.dvmax_values = control_limits;
    end
elseif strcmp(opts.modelType, 'doubleint')
    combined_results.acceleration_limits = control_limits;
    combined_results.dimensions = opts.dims;
    combined_results.disturbance_range = opts.dRange;
elseif strcmp(opts.modelType, 'dubinscar')
    combined_results.speed = opts.speed;
    combined_results.turning_limits = control_limits;
    combined_results.dimensions = opts.dims;
    combined_results.disturbance_range = opts.dRange;
end

save(fullfile(result_folder, [type_prefix, '_combined_results.mat']), '-struct', 'combined_results');

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
