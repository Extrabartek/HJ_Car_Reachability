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
generate_results = true;
visualize_results = true;
save_plots = false;

%% Define computation parameters
% Velocity values to test (m/s)
velocities = [20, 30];

% Maximum steering rate values to test (rad/s)
dvmax_values = [deg2rad(10), deg2rad(20)];

% Maximum simulation time (seconds)
tMax = 1.5;

% Time step for computation (seconds) 
dt = 0.05;

% Grid dimensions [gamma, beta, delta]
gridSize = [61, 51, 31];

% Grid minimum values (radians)
gridMin = [deg2rad(-150), deg2rad(-25), deg2rad(-10)];

% Grid maximum values (radians)
gridMax = [deg2rad(150), deg2rad(25), deg2rad(10)];

% Size of target set (radians)
targetSize = [deg2rad(15), deg2rad(6), deg2rad(22)];

% Control mode ('min' for target reaching, 'max' for target avoidance)
uMode = 'min';

%% Path to load existing results (if not generating new ones)
path_to_results_folder = '/home/bartosz/Documents/master_thesis/code_base/HJ_Car_Reachability/results/';
existing_result_folder = strcat(path_to_results_folder,'steered_brs_results_20250326_144608_vx30-30_dvmax10-10');

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
    if isempty(existing_result_folder)
        error('Must specify existing_result_folder when generate_results is false');
    end
    result_folder = existing_result_folder;
    disp(['Using existing results from: ', result_folder]);
end

%% Visualize results
if visualize_results
    disp('Starting visualization for the steered bicycle model...');
    visualize_brs_results_steered(result_folder, ...
        'plotType', {'slices', 'comparison'}, ... % Options: 'slices', 'comparison', 'detailed'
        'saveFigs', save_plots, ...
        'figFormat', 'png');

    disp('All processing completed successfully!');
    disp(['Results saved to: ', result_folder]);
end

end