function compute_frs_mzmax_wrapper()
% COMPUTE_FRS_MZMAX_WRAPPER Script to run FRS computations with user-defined parameters
%
% This wrapper script allows the user to easily configure and run Forward Reachable Set
% computations with different parameter combinations, and then automatically visualize 
% the results.
%
% The user can modify the parameter values below and run this script to
% perform the entire computation and visualization pipeline.

clear; clc;

%% Select if you want to load or produce results
generate_results = true;
visualize_results = true;
save_plots = true;

%% Define computation parameters
% Velocity values to test (m/s)
velocities = [30];

% Maximum yaw moment values to test (N·m)
mzmax_values = [15000];

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

% Control mode ('max' for forward reachability)
uMode = 'max';

if generate_results
    %% Run FRS computation
    disp('Starting FRS computation...');
    result_folder = compute_frs_mzmax(velocities, mzmax_values, ...
        'tMax', tMax, ...
        'dt', dt, ...
        'gridSize', gridSize, ...
        'gridMin', gridMin, ...
        'gridMax', gridMax, ...
        'targetSize', targetSize, ...
        'uMode', uMode);
else
    folder_base = '/home/bartosz/Documents/master_thesis/code_base/HJ_Car_Reachability/results/';
    result_folder = strcat(folder_base, 'frs_results_last_run'); % Change to your actual folder
end

%% Visualize results
if visualize_results
    disp('Computation complete. Starting visualization...');
    visualize_frs_results(result_folder, ... % Using FRS-specific visualization function
        'plotType', {'velocity_stack', 'control', 'comparison'}, ...
        'saveFigs', save_plots, ...
        'figFormat', 'png');

    disp('All processing completed successfully!');
    disp(['Results saved to: ', result_folder]);
end

end