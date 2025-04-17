 %% Car Trajectory Visualization Wrapper
% This script simplifies the process of loading and visualizing car trajectories
% It handles loading BRS data and trajectory computation with better error handling

% Clear workspace and close figures
clear;
clc;
close all;

%% Define folder paths to your BRS results
% Replace with your actual path to BRS results folder
main_results_folder = '/home/bartosz/Documents/master_thesis/code_base/HJ_Car_Reachability/results/';
brs_folder = fullfile(main_results_folder, 'steered_brs_results_20250416_154943_vx30-30_dvmax20-20'); 
                                                            

%% Define the initial state for trajectory computation
% Format: [gamma; beta; delta] (yaw rate, sideslip angle, steering angle) in radians
xinit = [deg2rad(0); deg2rad(-10); deg2rad(-19)];

%% Set parameters for trajectory computation and visualization
compute_new_trajectory = true;  % Set to false to load a previously saved trajectory
velocity_idx = 1;               % Index of velocity to use from BRS data
dv_max_idx = 1;                 % Index of steering rate limit to use
max_time = 6.0;                 % Maximum trajectory time (seconds)

% Car visualization parameters
car_length = 4.5;               % Car length in meters
car_width = 1.8;                % Car width in meters
wheel_base = 2.7;               % Distance between axles in meters
grid_size = 20;                 % Size of the grid in the car view
save_video = false;              % Whether to save as video
video_file = 'car_trajectory.avi';  % Using .avi extension for compatibility

%% Load or compute trajectory
try
    if compute_new_trajectory
        % Compute a new trajectory
        fprintf('Computing trajectory for initial state [%.1f°/s, %.1f°, %.1f°]...\n', ...
            xinit(1)*180/pi, xinit(2)*180/pi, xinit(3)*180/pi);
        
        [traj, traj_tau, traj_u, metrics] = compute_trajectory_steered_from_folders(brs_folder, xinit, ...
            'velocityIdx', velocity_idx, ...
            'dvMaxIdx', dv_max_idx, ...
            'visualize', false, ...
            'maxTime', max_time);
        
        % Save the computed trajectory
        save('trajectory_data.mat', 'traj', 'traj_tau', 'traj_u', 'metrics', 'xinit', 'brs_folder');
        fprintf('Trajectory computed and saved to trajectory_data.mat\n');
    else
        % Load a previously computed trajectory
        trajectory_file = 'trajectory_data.mat';
        
        if exist(trajectory_file, 'file')
            load(trajectory_file);
            fprintf('Loaded trajectory from %s\n', trajectory_file);
            
            % Update the BRS folder if it was changed
            if ~exist('brs_folder', 'var') || ~strcmp(brs_folder, brs_folder)
                fprintf('Note: Using BRS folder from script, not from saved data.\n');
            end
        else
            error('Trajectory file not found: %s', trajectory_file);
        end
    end
catch err
    fprintf('Error in trajectory computation: %s\n', err.message);
    return;
end

%% Load the BRS and target data for visualization
try
    % Load the BRS data
    combined_file = fullfile(brs_folder, 'brs_combined_results.mat');
    if ~exist(combined_file, 'file')
        error('BRS results file not found: %s', combined_file);
    end
    
    fprintf('Loading BRS data...\n');
    brs_data = load(combined_file);
    
    % Extract necessary data
    g = brs_data.g;
    data_brs = brs_data.all_data{velocity_idx, dv_max_idx};  % Using specified indices
    data0 = brs_data.data0;                                 % Target set
    vx = brs_data.velocities(velocity_idx);                 % Longitudinal velocity
catch err
    fprintf('Error loading BRS data: %s\n', err.message);
    return;
end

%% Create the visualization
try
    fprintf('Creating car trajectory visualization...\n');
    
    % Ensure the visualizeCarTrajectory function is available
    if ~exist('visualizeCarTrajectory', 'file')
        error(['The visualizeCarTrajectory function is not found in the MATLAB path. ', ...
               'Please ensure it is available.']);
    end
    
    visualizeCarTrajectory(traj, traj_tau, g, data_brs, data0, vx, ...
        'x0', 0, 'y0', 0, 'psi0', 0, ...
        'saveVideo', save_video, ...
        'videoFile', video_file, ...
        'carLength', car_length, ...
        'carWidth', car_width, ...
        'wheelBase', wheel_base, ...
        'gridSize', grid_size, ...
        'playSpeed', 1.0, ...
        'frameRate', 90);
    
    fprintf('Visualization complete.\n');
    if save_video
        fprintf('Video saved to: %s\n', video_file);
    end
catch err
    fprintf('Error in visualization: %s\n', err.message);
    
    % If the error is related to VideoWriter, try again without saving video
    if contains(err.message, 'VideoWriter') || contains(err.message, 'profile')
        fprintf('Attempting visualization without video recording...\n');
        try
            visualizeCarTrajectory(traj, traj_tau, g, data_brs, data0, vx, ...
                'x0', 0, 'y0', 0, 'psi0', 0, ...
                'saveVideo', false, ...
                'carLength', car_length, ...
                'carWidth', car_width, ...
                'wheelBase', wheel_base, ...
                'gridSize', grid_size, ...
                'playSpeed', 1.0);
            fprintf('Visualization completed without video recording.\n');
        catch inner_err
            fprintf('Visualization failed: %s\n', inner_err.message);
        end
    end
end