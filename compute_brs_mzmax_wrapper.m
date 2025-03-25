function compute_brs_mzmax_wrapper()
% COMPUTE_BRS_MZMAX_WRAPPER Script to run BRS computations with user-defined parameters
%
% This wrapper script allows the user to easily configure and run BRS computations
% with different parameter combinations, and then automatically visualize the results.
%
% The user can modify the parameter values below and run this script to
% perform the entire computation and visualization pipeline.

%% Define computation parameters
% Velocity values to test (m/s)
velocities = [15, 30, 45];

% Maximum yaw moment values to test (N·m)
mzmax_values = [5000, 10000];

% Maximum simulation time (seconds)
tMax = 1.0;

% Time step for computation (seconds) 
dt = 0.05;

% Grid dimensions [sideslip, yaw rate]
gridSize = [71, 71];

% Grid minimum values (radians)
gridMin = [deg2rad(-150), deg2rad(-25)];

% Grid maximum values (radians)
gridMax = [deg2rad(150), deg2rad(25)];

% Size of target set (radians)
targetSize = [deg2rad(15), deg2rad(6)];

% Control mode ('min' for target reaching, 'max' for target avoidance)
uMode = 'min';

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

%% Visualize results
disp('Computation complete. Starting visualization...');
visualize_brs_results(result_folder, ...
    'plotType', {'control', 'detailed', 'comparison'}, ...
    'saveFigs', true, ...
    'figFormat', 'png');

disp('All processing completed successfully!');
disp(['Results saved to: ', result_folder]);
end