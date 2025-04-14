function compute_frs_dvmax_wrapper()
% COMPUTE_FRS_DVMAX_WRAPPER Script to run FRS computations with user-defined parameters
% for the steered bicycle model with 3 states (gamma, beta, delta)
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
visualize_results = false;
save_plots = false;
threed_plot = false;

%% Define computation parameters
% Velocity values to test (m/s)
velocities = [30];

% Maximum steering rate values to test (rad/s)
dvmax_values = [deg2rad(20)];

% Maximum simulation time (seconds)
tMax = 2.0;

% Time step for computation (seconds) 
dt = 0.01;

% Grid dimensions [gamma, beta, delta]
gridSize = [141, 141, 71];

% Grid minimum values (radians)
gridMin = [deg2rad(-120), deg2rad(-25), deg2rad(-10)];

% Grid maximum values (radians)
gridMax = [deg2rad(120), deg2rad(25), deg2rad(10)];

% Size of target set (radians)
targetSize = [deg2rad(15), deg2rad(3), deg2rad(1)];

% Control mode ('max' for forward reachability)
uMode = 'max';

%% Path to load existing results (if not generating new ones)

%% Fill the path to the results' folder
result_folder = '';


if generate_results
    %% Run FRS computation
    disp('Starting FRS computation for the steered bicycle model...');
    result_folder = compute_frs_dvmax(velocities, dvmax_values, ...
        'tMax', tMax, ...
        'dt', dt, ...
        'gridSize', gridSize, ...
        'gridMin', gridMin, ...
        'gridMax', gridMax, ...
        'targetSize', targetSize, ...
        'uMode', uMode);
else
    % Use existing results folder
    if isempty(result_folder)
        error('Must specify existing_result_folder when generate_results is false');
    end
end

%% Visualize results
if visualize_results
    disp('Starting visualization for the steered bicycle model...');
    visualize_frs_results_steered(result_folder, ... % Using FRS-specific visualization function
        'plotType', {'slices', 'comparison', 'detailed'}, ...
        'saveFigs', save_plots, ...
        'figFormat', 'png');

    disp('All processing completed successfully!');
    disp(['Results saved to: ', result_folder]);
end

if threed_plot
    visualize_3d_reachable_sets(result_folder);
end

end