function compute_brs_dvmax_wrapper()
% COMPUTE_BRS_DVMAX_WRAPPER Script to run BRS computations with user-defined parameters
% for the steered bicycle model with 3 states (gamma, beta, delta)
%
% This wrapper script allows the user to easily configure and run BRS computations
% with different parameter combinations, and then automatically visualize the results.
%
% The user can modify the parameter values below and run this script to
% perform the entire computation and visualization pipeline.

clear; clc;

%% Select if you want to load or produce results
generate_results = false;
visualize_results = false;
save_plots = false;
threed_plot = true;

%% Define computation parameters
% Velocity values to test (m/s)
velocities = [30];

% Maximum steering rate values to test (rad/s)
dvmax_values = [deg2rad(20)];

% Maximum simulation time (seconds)
tMax = 2.5;

% Time step for computation (seconds) 
dt = 0.01;

% Grid dimensions [gamma, beta, delta]
gridSize = [101, 101, 51];

% Grid minimum values (radians)
gridMin = [deg2rad(-150), deg2rad(-25), deg2rad(-20)];

% Grid maximum values (radians)
gridMax = [deg2rad(150), deg2rad(25), deg2rad(20)];

% Size of target set (radians)
targetSize = [deg2rad(15), deg2rad(6), deg2rad(1)];

% Control mode ('min' for target reaching, 'max' for target avoidance)
uMode = 'min';

%% Path to load existing results (if not generating new ones)

%% Fill the path to the results' folder
result_folder = ''; 


if generate_results
    %% Run BRS computation
    disp('Starting BRS computation for the steered bicycle model...');
    result_folder = compute_brs_dvmax(velocities, dvmax_values, ...
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
    visualize_brs_results_steered(result_folder, ...
        'plotType', {'detailed', 'slices', 'comparison'}, ... % Options: 'slices', 'comparison', 'detailed'
        'saveFigs', save_plots, ...
        'figFormat', 'png');

    disp('All processing completed successfully!');
    disp(['Results saved to: ', result_folder]);
end

if threed_plot
    visualize_3d_reachable_sets(result_folder);
end

end