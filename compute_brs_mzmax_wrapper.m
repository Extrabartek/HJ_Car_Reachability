function compute_brs_mzmax_wrapper()
% COMPUTE_BRS_MZMAX_WRAPPER Script to run BRS computations with user-defined parameters
%
% This wrapper script allows the user to easily configure and run BRS computations
% with different parameter combinations, and then automatically visualize the results.
%
% The user can modify the parameter values below and run this script to
% perform the entire computation and visualization pipeline.

%% Select if you want to load or produce results
generate_results = false;

%% Define computation parameters
% Velocity values to test (m/s)
velocities = [30];

% Maximum yaw moment values to test (N·m)
mzmax_values = [5000];

% Maximum simulation time (seconds)
tMax = 0.5;

% Time step for computation (seconds) 
dt = 0.05;

% Grid dimensions [sideslip, yaw rate]
gridSize = [51, 51];

% Grid minimum values (radians)
gridMin = [deg2rad(-150), deg2rad(-25)];

% Grid maximum values (radians)
gridMax = [deg2rad(150), deg2rad(25)];

% Size of target set (radians)
targetSize = [deg2rad(10), deg2rad(4)];

% Control mode ('min' for target reaching, 'max' for target avoidance)
uMode = 'min';

if generate_results
    %% Run BRS computation
    disp('Starting BRS computation...');
    result_folder = compute_brs_mzmax(velocities, mzmax_values, ...
        'tMax', tMax, ...
        'dt', dt, ...
        'gridSize', gridSize, ...
        'gridMin', gridMin, ...
        'gridMax', gridMax, ...
        'targetSize', targetSize, ...
        'uMode', uMode);
else
    folder_base = '/home/bartosz/Documents/master_thesis/code_base/HJ_Car_Reachability/results/';
    result_folder = strcat(folder_base, 'brs_results_20250325_111921_vx10-30_mz1-5000');
%% Visualize results
disp('Computation complete. Starting visualization...');
visualize_brs_results(result_folder, ...
    'plotType', {'control', 'detailed', 'comparison'}, ...
    'saveFigs', true, ...
    'figFormat', 'png');

disp('All processing completed successfully!');
disp(['Results saved to: ', result_folder]);
end