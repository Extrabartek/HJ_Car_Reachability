%% Trajectory Calculator for Steered Bicycle Model
% This script demonstrates how to compute optimal trajectories for the steered bicycle model
% using backward and forward reachable sets.

% Clear workspace and close figures
clear;
clc;
close all;

%% Define folder paths (these should point to your actual result folders)
% Replace these paths with your actual BRS and FRS result folder paths
% main_results_folder = '/path/to/your/results/folder/';
% brs_folder = fullfile(main_results_folder, 'steered_brs_results_...');
% frs_folder = fullfile(main_results_folder, 'steered_frs_results_...');

% Example paths (for documentation only - adjust these to your actual paths)
main_results_folder = '/home/bartosz/Documents/master_thesis/code_base/HJ_Car_Reachability/results/';
brs_folder = fullfile(main_results_folder, 'steered_brs_results_20250415_161304_vx30-30_dvmax20-20');
frs_folder = fullfile(main_results_folder, 'steered_frs_results_20250402_122140_vx30-30_dvmax20-20');

%% Define an initial state
% Format: [gamma; beta; delta] (yaw rate, sideslip angle, steering angle) in radians
% Example: 10 deg/s yaw rate, 5 deg sideslip, 2 deg steering angle
xinit = [deg2rad(-120); deg2rad(-15); deg2rad(2)];

%% Define a custom target set (optional)
% If you want to define a custom target set, uncomment and modify these lines:
% use_custom_target = true;
% custom_target_size = [deg2rad(2), deg2rad(1), deg2rad(0.5)]; % [gamma, beta, delta] size
% target_center = [0, 0, 0]; % Target at origin

% Otherwise, to use the default target from BRS:
use_custom_target = false;
custom_target_size = [];
target_center = [0, 0, 0];

%% Set up computation parameters
velocity_idx = 1;         % Index of the velocity to use
dv_max_idx = 1;           % Index of the steering rate limit to use
use_frs = false;          % Whether to use FRS for safety constraints
frs_weight = 0.0;         % Weight for FRS constraints (0-1)
max_time = 2.5;           % Maximum trajectory time (seconds)
delta_slice = [];         % Which delta slice to visualize (default: middle slice)

%% Save options
save_results = true;                  % Whether to save trajectory data
save_plots = true;                    % Whether to save visualization plots
output_folder = brs_folder;           % Save results to the same folder as BRS data

%% Compute the optimal trajectory
[traj, traj_tau, traj_u, metrics] = compute_trajectory_steered_from_folders(brs_folder, xinit, ...
    'frs_folder', frs_folder, ...
    'useFRS', use_frs, ...
    'velocityIdx', velocity_idx, ...
    'dvMaxIdx', dv_max_idx, ...
    'frsWeight', frs_weight, ...
    'useTargetSet', ~use_custom_target, ...
    'customTargetSize', custom_target_size, ...
    'targetCenter', target_center, ...
    'visualize', true, ...
    'savePlots', save_plots, ...
    'maxTime', max_time, ...
    'deltaSlice', delta_slice);

% Display success message
fprintf('\nTrajectory computation completed successfully!\n');
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
%% Manual save trajectory data (if not using the built-in save option)
if save_results  
    % Create descriptive filename
    init_str = [num2str(xinit(1) * 180/pi, '%.1f'), '_', num2str(xinit(2) * 180/pi, '%.1f'), '_', num2str(xinit(3) * 180/pi, '%.1f')];
    filename = fullfile(output_folder, ['trajectory_', init_str, '.mat']);
    
    % Save trajectory data
    save(filename, 'traj', 'traj_tau', 'traj_u', 'metrics', 'xinit');
    fprintf('Saved trajectory data to %s\n', filename);
end
