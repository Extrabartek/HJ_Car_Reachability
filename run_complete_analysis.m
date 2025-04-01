function run_complete_analysis(model_type, varargin)
% RUN_COMPLETE_ANALYSIS Performs complete reachability analysis for a vehicle model
%
% This function runs both backward and forward reachability analysis and then
% compares the results, all in a single call.
%
% Inputs:
%   model_type    - Type of vehicle model: '2state' or '3state'
%   varargin      - Optional parameter-value pairs:
%                   'velocities'   - Array of velocities [m/s] (default depends on model)
%                   'controlLimits' - Array of control limits (default depends on model)
%                   'tMax'         - Maximum simulation time [s] (default: 1.0)
%                   'gridSize'     - Grid size (default depends on model)
%                   'targetSize'   - Size of target set (default depends on model)
%                   'savePlots'    - Whether to save plots (default: true)
%
% Example:
%   run_complete_analysis('2state', 'velocities', [15, 30], 'controlLimits', [3000, 5000]);
%   run_complete_analysis('3state', 'velocities', [20], 'controlLimits', [deg2rad(10)]);

%% Parse inputs
p = inputParser;
p.addRequired('model_type', @(x) any(strcmp(x, {'2state', '3state'})));
p.addParameter('velocities', [], @isnumeric);
p.addParameter('controlLimits', [], @isnumeric);
p.addParameter('tMax', 1.0, @isnumeric);
p.addParameter('gridSize', [], @isnumeric);
p.addParameter('targetSize', [], @isnumeric);
p.addParameter('savePlots', true, @islogical);

p.parse(model_type, varargin{:});
opts = p.Results;

%% Set default parameters based on model type
if strcmp(model_type, '2state')
    % Default parameters for 2-state model
    if isempty(opts.velocities)
        velocities = [20, 30];
    else
        velocities = opts.velocities;
    end
    
    if isempty(opts.controlLimits)
        mzmax_values = [3000, 5000];
    else
        mzmax_values = opts.controlLimits;
    end
    
    if isempty(opts.gridSize)
        gridSize = [71, 55];
    else
        gridSize = opts.gridSize;
    end
    
    gridMin = [deg2rad(-150), deg2rad(-25)];
    gridMax = [deg2rad(150), deg2rad(25)];
    
    if isempty(opts.targetSize)
        targetSize = [deg2rad(10), deg2rad(4)];
    else
        targetSize = opts.targetSize;
    end
    
else % 3-state model
    % Default parameters for 3-state model
    if isempty(opts.velocities)
        velocities = [20, 30];
    else
        velocities = opts.velocities;
    end
    
    if isempty(opts.controlLimits)
        dvmax_values = [deg2rad(10), deg2rad(20)];
    else
        dvmax_values = opts.controlLimits;
    end
    
    if isempty(opts.gridSize)
        gridSize = [61, 51, 31];
    else
        gridSize = opts.gridSize;
    end
    
    gridMin = [deg2rad(-150), deg2rad(-25), deg2rad(-10)];
    gridMax = [deg2rad(150), deg2rad(25), deg2rad(10)];
    
    if isempty(opts.targetSize)
        targetSize = [deg2rad(15), deg2rad(6), deg2rad(5)];
    else
        targetSize = opts.targetSize;
    end
end

%% Compute backward reachable sets
disp('*** STEP 1: Computing Backward Reachable Sets ***');

if strcmp(model_type, '2state')
    % 2-state model
    brs_folder = compute_brs_mzmax(velocities, mzmax_values, ...
        'tMax', opts.tMax, ...
        'gridSize', gridSize, ...
        'gridMin', gridMin, ...
        'gridMax', gridMax, ...
        'targetSize', targetSize, ...
        'uMode', 'min'); % min for BRS
else
    % 3-state model
    brs_folder = compute_brs_dvmax(velocities, dvmax_values, ...
        'tMax', opts.tMax, ...
        'gridSize', gridSize, ...
        'gridMin', gridMin, ...
        'gridMax', gridMax, ...
        'targetSize', targetSize, ...
        'uMode', 'min'); % min for BRS
end

%% Compute forward reachable sets
disp('*** STEP 2: Computing Forward Reachable Sets ***');

if strcmp(model_type, '2state')
    % 2-state model
    frs_folder = compute_frs_mzmax(velocities, mzmax_values, ...
        'tMax', opts.tMax, ...
        'gridSize', gridSize, ...
        'gridMin', gridMin, ...
        'gridMax', gridMax, ...
        'targetSize', targetSize, ...
        'uMode', 'max'); % max for FRS
else
    % 3-state model
    frs_folder = compute_frs_dvmax(velocities, dvmax_values, ...
        'tMax', opts.tMax, ...
        'gridSize', gridSize, ...
        'gridMin', gridMin, ...
        'gridMax', gridMax, ...
        'targetSize', targetSize, ...
        'uMode', 'max'); % max for FRS
end

%% Visualize results
disp('*** STEP 3: Visualizing BRS and FRS Results Separately ***');

if strcmp(model_type, '2state')
    % Visualize 2-state model results
    visualize_brs_results(brs_folder, ...
        'plotType', {'velocity_stack', 'control', 'comparison'}, ...
        'saveFigs', opts.savePlots, ...
        'figFormat', 'png');
    
    visualize_frs_results(frs_folder, ...
        'plotType', {'velocity_stack', 'control', 'comparison'}, ...
        'saveFigs', opts.savePlots, ...
        'figFormat', 'png');
else
    % Visualize 3-state model results
    visualize_brs_results_steered(brs_folder, ...
        'plotType', {'slices', 'comparison'}, ...
        'saveFigs', opts.savePlots, ...
        'figFormat', 'png');
    
    visualize_frs_results_steered(frs_folder, ...
        'plotType', {'slices', 'comparison'}, ...
        'saveFigs', opts.savePlots, ...
        'figFormat', 'png');
end

%% Compare BRS and FRS
disp('*** STEP 4: Comparing BRS and FRS Results ***');

visualize_brs_frs_comparison(brs_folder, frs_folder, ...
    'saveFigs', opts.savePlots, ...
    'figFormat', 'png', ...
    'model', model_type);

disp('*** Complete Analysis Finished Successfully ***');
disp(['BRS results saved to: ', brs_folder]);
disp(['FRS results saved to: ', frs_folder]);
end