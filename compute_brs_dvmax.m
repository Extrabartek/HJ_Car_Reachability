function result_folder = compute_brs_dvmax(velocities, dvmax_values, varargin)
% COMPUTE_BRS_DVMAX Computes backward reachable sets for vehicle with different
% steering rate limits and vehicle speeds
%
% Inputs:
%   velocities    - Array of longitudinal velocities to test [m/s]
%   dvmax_values  - Array of maximum steering rate values to test [rad/s]
%   varargin      - Optional parameter-value pairs:
%                   'tMax'      - Maximum simulation time [s] (default: 1.0)
%                   'dt'        - Time step [s] (default: 0.05)
%                   'gridSize'  - Grid size [nx, ny, nz] (default: [51, 51, 51])
%                   'gridMin'   - Grid minimum values [rad] (default: [-pi/2, -pi/4, -pi/12])
%                   'gridMax'   - Grid maximum values [rad] (default: [pi/2, pi/4, pi/12])
%                   'targetSize'- Size of target set [rad] (default: [pi/12, pi/30, pi/36])
%                   'uMode'     - Control mode ('min' or 'max') (default: 'min')
%
% Output:
%   result_folder - Path to the folder where results are saved
%
% Example:
%   result_folder = compute_brs_dvmax([15, 30], [deg2rad(5), deg2rad(10)], 'tMax', 0.5);

%% Parse inputs
p = inputParser;
p.addRequired('velocities', @isnumeric);
p.addRequired('dvmax_values', @isnumeric);
p.addParameter('tMax', 1.0, @isnumeric);
p.addParameter('dt', 0.05, @isnumeric);
p.addParameter('gridSize', [51, 51, 51], @isnumeric);
p.addParameter('gridMin', [deg2rad(-150), deg2rad(-25), deg2rad(-10)], @isnumeric);
p.addParameter('gridMax', [deg2rad(150), deg2rad(25), deg2rad(10)], @isnumeric);
p.addParameter('targetSize', [deg2rad(15), deg2rad(6), deg2rad(5)], @isnumeric);
p.addParameter('uMode', 'min', @ischar);

p.parse(velocities, dvmax_values, varargin{:});
opts = p.Results;

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
folder_name = sprintf('steered_brs_results_%s', timestamp);

% Create descriptive folder name with parameters
param_desc = sprintf('_vx%d-%d_dvmax%d-%d', ...
    min(velocities), max(velocities), ...
    round(min(dvmax_values) * 180/pi), round(max(dvmax_values) * 180/pi));
    
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
data0 = shapeRectangleByCenter(g, [0; 0; 0], target_size);

%% Setup base parameters
% Base parameters for steered model: [m, Vx, Lf, Lr, Iz, mu, dv_max, Cf, Cr]
base_params = [1430; 50; 1.05; 1.61; 2059.2; 1; deg2rad(540); 90700; 109000];

%% Store simulation parameters
sim_params = struct();
sim_params.velocities = velocities;
sim_params.dvmax_values = dvmax_values;
sim_params.grid_min = grid_min;
sim_params.grid_max = grid_max;
sim_params.grid_size = N;
sim_params.tau = tau;
sim_params.base_params = base_params;
sim_params.target_size = target_size;
sim_params.uMode = opts.uMode;
sim_params.timestamp = timestamp;

% Save simulation parameters
save(fullfile(result_folder, 'sim_params.mat'), 'sim_params');

%% Initialize storage structures
all_data = cell(length(velocities), length(dvmax_values));
all_data_full = cell(length(velocities), length(dvmax_values));
control_data = cell(length(velocities), length(dvmax_values));

%% Loop through each velocity and steering rate combination
for v_idx = 1:length(velocities)
    % Update the velocity parameter
    params = base_params;
    params(2) = velocities(v_idx);
    
    for d_idx = 1:length(dvmax_values)
        % Update the steering rate parameter
        params(7) = dvmax_values(d_idx);   % dv_max
        
        % Display progress
        fprintf('Computing BRS for velocity = %d m/s, dv_max = %.2f rad/s (%.2f deg/s)...\n', ...
                velocities(v_idx), dvmax_values(d_idx), dvmax_values(d_idx) * 180/pi);
        
        % Define dynamic system with current velocity and dv_max
        dCar = NonlinearBicycleSteered([0; 0; 0], params);
        
        % Put grid and dynamic systems into schemeData
        schemeData.grid = g;
        schemeData.dynSys = dCar;
        schemeData.accuracy = 'high'; % Set accuracy
        schemeData.uMode = opts.uMode;
        
        % Setup visualization options
        HJIextraArgs.visualize.valueSet = true;
        HJIextraArgs.visualize.initialValueSet = true;
        HJIextraArgs.visualize.figNum = 1;
        HJIextraArgs.visualize.deleteLastPlot = true;
        
        % Solve HJ PDE to get backward reachable sets
        [data, tau2, extraOuts] = HJIPDE_solve(data0, tau, schemeData, 'set', HJIextraArgs);
        
        % Store the computed data
        all_data{v_idx, d_idx} = data(:,:,:,end);
        all_data_full{v_idx, d_idx} = data;
        
        % Calculate optimal control
        % Get gradient of value function (for the delta dimension)
        derivs = extractCostates3D(g, data(:,:,:,end));
        
        % Initialize control grid
        control_grid = zeros(size(g.xs{1}));
        
        % Determine optimal control based on the sign of the derivative
        if strcmp(opts.uMode, 'min')
            % For target reaching:
            control_grid(derivs{3} >= 0) = -params(7);  % -dv_max
            control_grid(derivs{3} < 0) = params(7);    % dv_max
        else
            % For target avoiding:
            control_grid(derivs{3} >= 0) = params(7);   % dv_max
            control_grid(derivs{3} < 0) = -params(7);   % -dv_max
        end
        
        % Store the control grid
        control_data{v_idx, d_idx} = control_grid;
        
        % Save individual result file for each combination
        result_filename = sprintf('brs_v%d_dvmax%.0f.mat', velocities(v_idx), dvmax_values(d_idx) * 180/pi);
        save(fullfile(result_folder, result_filename), ...
             'g', 'data0', 'data', 'control_grid', 'params', 'tau');
        
        fprintf('Saved result to %s\n', fullfile(result_folder, result_filename));
    end
end

% Save combined results
disp('Saving combined results...');
save(fullfile(result_folder, 'brs_combined_results.mat'), ...
     'g', 'data0', 'all_data', 'all_data_full', 'control_data', ...
     'velocities', 'dvmax_values', 'tau', 'base_params');

disp('Computation complete!');
disp(['Results saved to: ', result_folder]);

end

% Helper function to extract costates (gradients) from the 3D value function
function derivs = extractCostates3D(g, data)
    % Initialize derivatives
    derivs = cell(g.dim, 1);
    
    % Use upwinding for proper gradient calculation
    for i = 1:g.dim
        % Initialize arrays for left and right derivatives
        deriv_left = zeros(size(data));
        deriv_right = zeros(size(data));
        
        % Calculate derivatives using central differences
        % For a 3D grid, need to handle derivatives along each dimension
        switch i
            case 1 % First dimension (gamma)
                % Interior points
                deriv_left(2:end,:,:) = (data(2:end,:,:) - data(1:end-1,:,:)) / g.dx(i);
                deriv_right(1:end-1,:,:) = (data(2:end,:,:) - data(1:end-1,:,:)) / g.dx(i);
                
                % Boundary points
                deriv_left(1,:,:) = (data(2,:,:) - data(1,:,:)) / g.dx(i);
                deriv_right(end,:,:) = (data(end,:,:) - data(end-1,:,:)) / g.dx(i);
                
            case 2 % Second dimension (beta)
                % Interior points
                deriv_left(:,2:end,:) = (data(:,2:end,:) - data(:,1:end-1,:)) / g.dx(i);
                deriv_right(:,1:end-1,:) = (data(:,2:end,:) - data(:,1:end-1,:)) / g.dx(i);
                
                % Boundary points
                deriv_left(:,1,:) = (data(:,2,:) - data(:,1,:)) / g.dx(i);
                deriv_right(:,end,:) = (data(:,end,:) - data(:,end-1,:)) / g.dx(i);
                
            case 3 % Third dimension (delta)
                % Interior points
                deriv_left(:,:,2:end) = (data(:,:,2:end) - data(:,:,1:end-1)) / g.dx(i);
                deriv_right(:,:,1:end-1) = (data(:,:,2:end) - data(:,:,1:end-1)) / g.dx(i);
                
                % Boundary points
                deriv_left(:,:,1) = (data(:,:,2) - data(:,:,1)) / g.dx(i);
                deriv_right(:,:,end) = (data(:,:,end) - data(:,:,end-1)) / g.dx(i);
        end
        
        % Choose appropriate derivative based on the Hamiltonian maximization/minimization
        if i == 3  % For steering control (delta dimension)
            % For minimizing Hamiltonian, use the derivative that gives the larger value
            derivs{i} = deriv_left;
            derivs{i}(deriv_right < deriv_left) = deriv_right(deriv_right < deriv_left);
        else
            % For other dimensions, use central differencing
            derivs{i} = (deriv_left + deriv_right) / 2;
        end
    end
end