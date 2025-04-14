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
brs_folder = fullfile(main_results_folder, 'steered_brs_results_20250402_091041_vx30-30_dvmax20-20');
frs_folder = fullfile(main_results_folder, 'steered_frs_results_20250402_122140_vx30-30_dvmax20-20');

%% Define an initial state
% Format: [gamma; beta; delta] (yaw rate, sideslip angle, steering angle) in radians
% Example: 10 deg/s yaw rate, 5 deg sideslip, 2 deg steering angle
xinit = [deg2rad(-130); deg2rad(-13); deg2rad(0)];

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
frs_weight = 1;         % Weight for FRS constraints (0-1)
max_time = 2.5;           % Maximum trajectory time (seconds)
delta_slice = [];         % Which delta slice to visualize (default: middle slice)

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
        'savePlots', false, ...
        'maxTime', max_time, ...
        'deltaSlice', delta_slice);
    
    % Display success message
    fprintf('\nTrajectory computation completed successfully!\n');
   

%% Additional Examples (commented out)
% To run these examples, uncomment the sections of interest

%% Example 1: Computing trajectory to a custom target set
% xinit_ex1 = [deg2rad(15); deg2rad(-3); deg2rad(5)];
% custom_target_size_ex1 = [deg2rad(1), deg2rad(0.5), deg2rad(0.2)];
% target_center_ex1 = [0, 0, 0];
% 
% [traj_ex1, traj_tau_ex1, traj_u_ex1, metrics_ex1] = compute_trajectory_steered_from_folders(brs_folder, xinit_ex1, ...
%     'useTargetSet', false, ...
%     'customTargetSize', custom_target_size_ex1, ...
%     'targetCenter', target_center_ex1, ...
%     'velocityIdx', velocity_idx, ...
%     'dvMaxIdx', dv_max_idx, ...
%     'visualize', true, ...
%     'figNum', 2);

%% Example 2: Using FRS for safety constraints
% xinit_ex2 = [deg2rad(-10); deg2rad(10); deg2rad(-2)];
% 
% [traj_ex2, traj_tau_ex2, traj_u_ex2, metrics_ex2] = compute_trajectory_steered_from_folders(brs_folder, xinit_ex2, ...
%     'frs_folder', frs_folder, ...
%     'useFRS', true, ...
%     'frsWeight', 0.7, ...
%     'velocityIdx', velocity_idx, ...
%     'dvMaxIdx', dv_max_idx, ...
%     'visualize', true, ...
%     'figNum', 3);

%% Example 3: Comparing multiple trajectories from different initial states
% xinit_set = {
%     [deg2rad(20); deg2rad(5); deg2rad(0)],   % Initial state 1
%     [deg2rad(-20); deg2rad(-5); deg2rad(0)], % Initial state 2
%     [deg2rad(0); deg2rad(10); deg2rad(5)]    % Initial state 3
% };
% 
% % Create a figure for comparison
% figure(10);
% colors = {'b', 'r', 'g'};
% 
% % Compute and plot trajectories
% for i = 1:length(xinit_set)
%     try
%         % Compute trajectory (no visualization)
%         [traj_comp, traj_tau_comp, ~, ~] = compute_trajectory_steered_from_folders(brs_folder, xinit_set{i}, ...
%             'velocityIdx', velocity_idx, ...
%             'dvMaxIdx', dv_max_idx, ...
%             'visualize', false);
%         
%         % Plot in 3D
%         plot3(traj_comp(2,:)*180/pi, traj_comp(1,:)*180/pi, traj_comp(3,:)*180/pi, ...
%             'Color', colors{i}, 'LineWidth', 2);
%         hold on;
%         plot3(traj_comp(2,1)*180/pi, traj_comp(1,1)*180/pi, traj_comp(3,1)*180/pi, ...
%             'o', 'Color', colors{i}, 'MarkerSize', 8, 'MarkerFaceColor', colors{i});
%         
%         fprintf('Trajectory %d computed successfully.\n', i);
%     catch
%         fprintf('Could not compute trajectory %d.\n', i);
%     end
% end
% 
% % Finalize the comparison plot
% grid on;
% xlabel('Sideslip Angle (deg)');
% ylabel('Yaw Rate (deg/s)');
% zlabel('Steering Angle (deg)');
% title('Comparison of Trajectories from Different Initial States');
% view([-30, 30]);
% legend('Trajectory 1', 'Initial 1', 'Trajectory 2', 'Initial 2', 'Trajectory 3', 'Initial 3');