function result_folder = compute_brs_mzmax(velocities, mzmax_values, varargin)
% COMPUTE_BRS_MZMAX Computes backward reachable sets for vehicle with different
% yaw moment control limits and vehicle speeds
%
% Inputs:
%   velocities    - Array of longitudinal velocities to test [m/s]
%   mzmax_values  - Array of maximum yaw moment values to test [N·m]
%   varargin      - Optional parameter-value pairs:
%                   'tMax'      - Maximum simulation time [s] (default: 1.0)
%                   'dt'        - Time step [s] (default: 0.05)
%                   'gridSize'  - Grid size [nx, ny] (default: [71, 71])
%                   'gridMin'   - Grid minimum values [rad] (default: [-pi/2, -pi/2])
%                   'gridMax'   - Grid maximum values [rad] (default: [pi/2, pi/2])
%                   'targetSize'- Size of target set [rad] (default: [pi/12, pi/30])
%                   'uMode'     - Control mode ('min' or 'max') (default: 'min')
%
% Output:
%   result_folder - Path to the folder where results are saved
%
% Example:
%   result_folder = compute_brs_mzmax([15, 30], [5000, 10000], 'tMax', 0.5);

%% Parse inputs
p = inputParser;
p.addRequired('velocities', @isnumeric);
p.addRequired('mzmax_values', @isnumeric);
p.addParameter('tMax', 1.0, @isnumeric);
p.addParameter('dt', 0.05, @isnumeric);
p.addParameter('gridSize', [71, 71], @isnumeric);
p.addParameter('gridMin', [deg2rad(-150), deg2rad(-25)], @isnumeric);
p.addParameter('gridMax', [deg2rad(150), deg2rad(25)], @isnumeric);
p.addParameter('targetSize', [deg2rad(15), deg2rad(6)], @isnumeric);
p.addParameter('uMode', 'min', @ischar);

p.parse(velocities, mzmax_values, varargin{:});
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
folder_name = sprintf('brs_results_%s', timestamp);

% Create descriptive folder name with parameters
param_desc = sprintf('_vx%d-%d_mz%d-%d', ...
    min(velocities), max(velocities), ...
    min(mzmax_values), max(mzmax_values));
    
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
data0 = shapeRectangleByCenter(g, [0; 0], target_size);

%% Setup base parameters
% Base parameters: [m, Vx, Lf, Lr, Iz, mu, Mzmax, Mzmin, Cf, Cr]
base_params = [1430; 50; 1.05; 1.61; 2059.2; 1; 10000; -10000; 90700; 109000];

%% Store simulation parameters
sim_params = struct();
sim_params.velocities = velocities;
sim_params.mzmax_values = mzmax_values;
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
all_data = cell(length(velocities), length(mzmax_values));
all_data_full = cell(length(velocities), length(mzmax_values));
control_data = cell(length(velocities), length(mzmax_values));

%% Loop through each velocity and yaw moment combination
for v_idx = 1:length(velocities)
    % Update the velocity parameter
    params = base_params;
    params(2) = velocities(v_idx);
    
    for m_idx = 1:length(mzmax_values)
        % Update the Mzmax and Mzmin parameters
        params(7) = mzmax_values(m_idx);   % Mzmax
        params(8) = -mzmax_values(m_idx);  % Mzmin (negative of Mzmax)
        
        % Display progress
        fprintf('Computing BRS for velocity = %d m/s, Mzmax = %d N·m...\n', ...
                velocities(v_idx), mzmax_values(m_idx));
        
        % Define dynamic system with current velocity and Mzmax
        dCar = NonlinearBicycle([0; 0], params);
        
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
        all_data{v_idx, m_idx} = data(:,:,end);
        all_data_full{v_idx, m_idx} = data;
        
        % Calculate optimal control
        % Get gradient of value function
        derivs = extractCostates(g, data(:,:,end));
        
        % Initialize control grid
        control_grid = zeros(size(g.xs{1}));
        
        % Determine optimal control based on the sign of the derivative
        if strcmp(opts.uMode, 'min')
            % For target reaching:
            control_grid(derivs{2} >= 0) = params(8);  % Mzmin
            control_grid(derivs{2} < 0) = params(7);   % Mzmax
        else
            % For target avoiding:
            control_grid(derivs{2} >= 0) = params(7);  % Mzmax
            control_grid(derivs{2} < 0) = params(8);   % Mzmin
        end
        
        % Store the control grid
        control_data{v_idx, m_idx} = control_grid;
        
        % Save individual result file for each combination
        result_filename = sprintf('brs_v%d_mz%d.mat', velocities(v_idx), mzmax_values(m_idx));
        save(fullfile(result_folder, result_filename), ...
             'g', 'data0', 'data', 'control_grid', 'params', 'tau');
        
        fprintf('Saved result to %s\n', fullfile(result_folder, result_filename));
    end
end

% Save combined results
disp('Saving combined results...');
save(fullfile(result_folder, 'brs_combined_results.mat'), ...
     'g', 'data0', 'all_data', 'all_data_full', 'control_data', ...
     'velocities', 'mzmax_values', 'tau', 'base_params');

disp('Computation complete!');
disp(['Results saved to: ', result_folder]);

end

% Helper function to extract costates (gradients) from the value function
function derivs = extractCostates(g, data)
    % Initialize derivatives
    derivs = cell(g.dim, 1);
    
    % Use upwinding for proper gradient calculation
    for i = 1:g.dim
        % Initialize arrays for left and right derivatives
        deriv_left = zeros(size(data));
        deriv_right = zeros(size(data));
        
        % Calculate left and right derivatives with appropriate boundary handling
        if i == 1
            % For the first dimension (beta)
            deriv_left(2:end, :) = (data(2:end, :) - data(1:end-1, :)) / g.dx(i);
            deriv_right(1:end-1, :) = (data(2:end, :) - data(1:end-1, :)) / g.dx(i);
            
            % Handle boundaries with one-sided differences
            deriv_left(1, :) = (data(2, :) - data(1, :)) / g.dx(i);
            deriv_right(end, :) = (data(end, :) - data(end-1, :)) / g.dx(i);
        else
            % For the second dimension (gamma)
            deriv_left(:, 2:end) = (data(:, 2:end) - data(:, 1:end-1)) / g.dx(i);
            deriv_right(:, 1:end-1) = (data(:, 2:end) - data(:, 1:end-1)) / g.dx(i);
            
            % Handle boundaries with one-sided differences
            deriv_left(:, 1) = (data(:, 2) - data(:, 1)) / g.dx(i);
            deriv_right(:, end) = (data(:, end) - data(:, end-1)) / g.dx(i);
        end
        
        % Choose derivative based on the sign of the Hamiltonian
        % For target reaching, take the derivative that would lead to a larger Hamiltonian value
        if i == 1  % beta dimension
            % The sign would depend on the system dynamics - for simplicity,
            % we'll use central differencing here
            derivs{i} = (deriv_left + deriv_right) / 2;
        else  % gamma dimension - critical for control determination
            derivs{i} = deriv_left;
            derivs{i}(deriv_right < deriv_left) = deriv_right(deriv_right < deriv_left);
        end
    end
end