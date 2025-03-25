function compute_brs_mzmax_wrapper()
% COMPUTE_BRS_MZMAX_WRAPPER Script to run BRS computations with user-defined parameters
%
% This wrapper script allows the user to easily configure and run BRS computations
% with different parameter combinations, and then automatically visualize the results.
%
% The user can modify the parameter values below and run this script to
% perform the entire computation and visualization pipeline.

clear; clc;

%% Select if you want to load or produce results
generate_results = false;
visualize_results = true;
save_plots = true;

%% Define computation parameters
% Velocity values to test (m/s)
velocities = 5:1:45;

% Maximum yaw moment values to test (N·m)
mzmax_values = [0, 3000];

% Maximum simulation time (seconds)
tMax = 1.0;

% Time step for computation (seconds) 
dt = 0.05;

% Grid dimensions [yaw rate, side slip]
gridSize = [71, 55];

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
    result_folder = strcat(folder_base, 'brs_results_20250325_172259_vx5-45_mz0-3000');
end
%% Visualize results
if visualize_results
    disp('Computation complete. Starting visualization...');
    visualize_brs_results(result_folder, ...
        'plotType', {'velocity_stack'}, ...
        'saveFigs', save_plots, ...
        'figFormat', 'png');

    disp('All processing completed successfully!');
    disp(['Results saved to: ', result_folder]);
end

end