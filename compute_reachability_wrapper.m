function compute_reachability_wrapper()
% COMPUTE_REACHABILITY_WRAPPER Script to run reachability computations with user-defined parameters
%
% This wrapper script allows you to easily configure and run reachability computations
% with different parameter combinations using the unified compute_reachable_set function.
%
% To use:
% 1. Modify the configuration parameters in the section below
% 2. Run the script
%
% The script will run the computation and visualize the results as specified.

clear; clc;

profile on;

%% Configuration Parameters - MODIFY THESE VALUES
% ---------------------------------------------------

%% Select if you want to load or produce results
generate_results = true;     % Set to false to load existing results
visualize_results = false;    % Set to false to skip visualization
save_plots = false;          % Set to true to save visualization figures

%% Model and computation type
modelType = 'bicycle';      % Options: 'bicycle', 'doubleInt', 'dubinsCar'
direction = 'backward';      % Options: 'backward' (BRS) or 'forward' (FRS)
controlType = 'mz';          % Options: 'mz' (yaw moment) or 'dv' (steering rate)
                             % (only used for bicycle model)

%% Model-specific parameters

% Bicycle model parameters
velocities = [20];           % Vehicle velocities to test [m/s] (for bicycle model)

% Double Integrator parameters (if modelType = 'doubleInt')
doubleint_dims = 1:2;        % Dimensions to use
doubleint_drange = [0, 0];   % Disturbance range
 
% Dubins Car parameters (if modelType = 'dubinsCar')
dubins_speed = 1;            % Constant speed [m/s]
dubins_dims = 1:3;           % Dimensions to use
dubins_drange = {[0;0;0], [0;0;0]}; % Disturbance range

%% Control limits (meaning depends on model type)
% - If modelType = 'bicycle' and controlType = 'mz': Max yaw moments [N·m]
% - If modelType = 'bicycle' and controlType = 'dv': Max steering rates [deg/s]
% - If modelType = 'doubleInt': Acceleration limits [m/s²]
% - If modelType = 'dubinsCar': Turning rate limits [deg/s]
control_limits = [1];       % Will be converted to radians for 'dv' and 'dubinsCar'

%% Time parameters
tMax = 1.00;                  % Maximum simulation time [s]
dt = 0.001;                   % Time step [s]

%% Grid parameters
% Grid size: Number of grid points in each dimension
gridSize = [301, 301];               % Empty = use defaults based on model type
                             % Default for 'bicycle'+'mz': [71, 71]
                             % Default for 'bicycle'+'dv': [51, 51, 51]
                             % Default for 'doubleInt': [101, 101]
                             % Default for 'dubinsCar': [51, 51, 51]

% Grid limits in degrees (will be converted to radians for angles)
gridMin_deg = [-500, -80];            % Empty = use defaults based on model type
gridMax_deg = [500, 80];            % Empty = use defaults based on model type

% Target set size in degrees (will be converted to radians for angles)
targetSize_deg = [20, 3];         % Empty = use defaults based on model type

%% Advanced parameters
uMode = '';                  % Control strategy: 'min', 'max', or '' (empty = use default)
                             % Default for 'backward': 'min'
                             % Default for 'forward': 'max'

accuracy = 'veryHigh';           % Numerical accuracy: 'low', 'medium', 'high', 'veryhigh'               

%% Path to existing results folder (only used if generate_results = false)
main_results_folder = '/home/bartosz/Documents/master_thesis/code_base/HJ_Car_Reachability/results/';
existing_result_folder = strcat(main_results_folder, 'doubleint_brs_results_20250516_100424_acc1-1');

%% Visualization options
% Types of plots to generate (cell array of strings)
% Options depend on model type - defaults will be used if empty
plot_types = {};

% ---------------------------------------------------
% END OF CONFIGURATION PARAMETERS
% ---------------------------------------------------

%% Process control limits based on model type
% Convert angle-based control limits from degrees to radians
if (strcmp(modelType, 'bicycle') && strcmp(controlType, 'dv')) || ...
   strcmp(modelType, 'dubinsCar')
    control_limits = deg2rad(control_limits);
end

%% Process model-specific parameters
model_params = struct();

if strcmp(modelType, 'bicycle')
    % For bicycle model, velocities and control type are needed
    model_params.velocities = velocities;
    model_params.controlType = controlType;
elseif strcmp(modelType, 'doubleInt')
    % For Double Integrator, set dimensions and disturbance range
    model_params.dims = doubleint_dims;
    model_params.dRange = doubleint_drange;
elseif strcmp(modelType, 'dubinsCar')
    % For Dubins Car, set speed, dimensions, and disturbance range
    model_params.speed = dubins_speed;
    model_params.dims = dubins_dims;
    model_params.dRange = dubins_drange;
end

%% Convert grid limits from degrees to radians if needed
if ~isempty(gridMin_deg) && ~isempty(gridMax_deg)
    % For angle dimensions, convert from degrees to radians
    if strcmp(modelType, 'bicycle') || strcmp(modelType, 'dubinsCar')
        % Find angle dimensions based on model type
        if strcmp(modelType, 'bicycle')
            if strcmp(controlType, 'mz')
                angle_dims = 1:2;  % Both dimensions are angles
            else
                angle_dims = 1:3;  % All three dimensions are angles
            end
        elseif strcmp(modelType, 'dubinsCar')
            angle_dims = [3];      % Only heading is an angle
        end
        
        % Convert those dimensions from degrees to radians
        gridMin = gridMin_deg;
        gridMax = gridMax_deg;
        
        for dim = angle_dims
            if dim <= length(gridMin)
                gridMin(dim) = deg2rad(gridMin_deg(dim));
                gridMax(dim) = deg2rad(gridMax_deg(dim));
            end
        end
    else
        % For Double Integrator, no conversion needed
        gridMin = gridMin_deg;
        gridMax = gridMax_deg;
    end
    
    model_params.gridMin = gridMin;
    model_params.gridMax = gridMax;
end

%% Convert target size from degrees to radians if needed
if ~isempty(targetSize_deg)
    % For angle dimensions, convert from degrees to radians
    if strcmp(modelType, 'bicycle') || strcmp(modelType, 'dubinsCar')
        % Find angle dimensions based on model type
        if strcmp(modelType, 'bicycle')
            if strcmp(controlType, 'mz')
                angle_dims = 1:2;  % Both dimensions are angles
            else
                angle_dims = 1:3;  % All three dimensions are angles
            end
        elseif strcmp(modelType, 'dubinsCar')
            angle_dims = [3];      % Only heading is an angle
        end
        
        % Convert those dimensions from degrees to radians
        targetSize = targetSize_deg;
        
        for dim = angle_dims
            if dim <= length(targetSize)
                targetSize(dim) = deg2rad(targetSize_deg(dim));
            end
        end
    else
        % For Double Integrator, no conversion needed
        targetSize = targetSize_deg;
    end
    
    model_params.targetSize = targetSize;
end

%% Display configuration summary
fprintf('\n=== Reachability Computation Configuration ===\n');
fprintf('Model Type: %s\n', modelType);
fprintf('Computation: %s (%s direction)\n', upper(direction), direction);

if strcmp(modelType, 'bicycle')
    fprintf('Control Type: %s\n', controlType);
    fprintf('Velocities: %s m/s\n', mat2str(velocities));
    
    if strcmp(controlType, 'mz')
        fprintf('Yaw Moment Limits: %s N·m\n', mat2str(control_limits));
    else
        fprintf('Steering Rate Limits: %s deg/s\n', mat2str(rad2deg(control_limits)));
    end
elseif strcmp(modelType, 'doubleInt')
    fprintf('Double Integrator with acceleration limits: %s m/s²\n', mat2str(control_limits));
    fprintf('Dimensions: %s\n', mat2str(doubleint_dims));
    fprintf('Disturbance range: %s\n', mat2str(doubleint_drange));
elseif strcmp(modelType, 'dubinsCar')
    fprintf('Dubins Car with speed: %g m/s\n', dubins_speed);
    fprintf('Turning rate limits: %s deg/s\n', mat2str(rad2deg(control_limits)));
    fprintf('Dimensions: %s\n', mat2str(dubins_dims));
end

if ~isempty(gridSize)
    fprintf('Grid Size: %s\n', mat2str(gridSize));
end
if ~isempty(gridMin_deg) && ~isempty(gridMax_deg)
    fprintf('Grid Limits: %s to %s\n', mat2str(gridMin_deg), mat2str(gridMax_deg));
end
if ~isempty(targetSize_deg)
    fprintf('Target Size: %s\n', mat2str(targetSize_deg));
end

fprintf('Time Parameters: tMax = %.2f s, dt = %.3f s\n', tMax, dt);
fprintf('Control Strategy: %s\n', uMode);
fprintf('Accuracy Level: %s\n', accuracy);
fprintf('=============================================\n\n');

% Initialize result folder variable
result_folder = '';

%% Run computation or load existing results
if generate_results
    fprintf('Starting %s computation for %s model...\n', upper(direction), modelType);
    
    % Prepare computation options
    compute_options = model_params;
    compute_options.direction = direction;
    compute_options.modelType = modelType;
    compute_options.tMax = tMax;
    compute_options.dt = dt;
    
    if ~isempty(gridSize)
        compute_options.gridSize = gridSize;
    end
    
    if ~isempty(uMode)
        compute_options.uMode = uMode;
    end
    
    compute_options.accuracy = accuracy;
    compute_options.visualize = true;
    
    % Call the consolidated compute_reachable_set function with complete options
    result_folder = compute_reachable_set(velocities, control_limits, compute_options);
else
    % Load existing results
    if isempty(existing_result_folder) || ~exist(existing_result_folder, 'dir')
        error('For loading existing results, you must specify a valid existing_result_folder');
    end
    
    result_folder = existing_result_folder;
    fprintf('Using existing results from: %s\n', result_folder);
end

%% Visualize results if requested
if visualize_results && ~isempty(result_folder)
    fprintf('Generating visualizations...\n');
    
    % Try to load simulation parameters to determine model type
    sim_params_file = fullfile(result_folder, 'sim_params.mat');
    if exist(sim_params_file, 'file')
        sim_params = load(sim_params_file);
        if isfield(sim_params, 'sim_params') && isfield(sim_params.sim_params, 'modelType')
            vis_model_type = sim_params.sim_params.modelType;
        else
            % Infer model type from folder name
            [~, folder_name] = fileparts(result_folder);
            if contains(folder_name, 'doubleint')
                vis_model_type = 'doubleInt';
            elseif contains(folder_name, 'dubinscar')
                vis_model_type = 'dubinsCar';
            else
                vis_model_type = 'bicycle';
            end
        end
    else
        % Default to the current model type
        vis_model_type = modelType;
    end
    
    % Set up visualization options
    viz_options = struct();
    viz_options.plotType = plot_types;
    viz_options.saveFigs = save_plots;
    viz_options.figFormat = 'png';
    viz_options.controlIdx = 1;  % Use first control limit by default
    viz_options.velocityIdx = 1; % Use first velocity by default
    
    % Check if unified visualization function exists
    if exist('visualize_reachability_results', 'file')
        try
            % For bicycle model, use the unified visualization function
            if strcmp(vis_model_type, 'bicycle')
                visualize_reachability_results(result_folder, viz_options);
            elseif strcmp(vis_model_type, 'doubleInt')
                % For Double Integrator, use a specialized visualization function if available
                if exist('visualize_doubleint_results', 'file')
                    visualize_doubleint_results(result_folder, viz_options);
                else
                    % Create a simple visualization
                    visualize_double_integrator_simple(result_folder);
                end
            elseif strcmp(vis_model_type, 'dubinsCar')
                % For Dubins Car, use a specialized visualization function if available
                if exist('visualize_dubinscar_results', 'file')
                    visualize_dubinscar_results(result_folder, viz_options);
                else
                    % Create a simple visualization
                    visualize_dubins_car_simple(result_folder);
                end
            end
        catch vis_err
            fprintf('Visualization error: %s\n', vis_err.message);
            fprintf('Falling back to basic visualization.\n');
            
            % Load and display basic results
            combined_file = fullfile(result_folder, 'brs_combined_results.mat');
            if ~exist(combined_file, 'file')
                combined_file = fullfile(result_folder, 'frs_combined_results.mat');
            end
            
            if exist(combined_file, 'file')
                data = load(combined_file);
                
                % Display based on model dimensions
                if strcmp(vis_model_type, 'doubleInt') || ...
                   (strcmp(vis_model_type, 'bicycle') && ~isfield(data, 'dvmax_values'))
                    % 2D visualization
                    figure;
                    if isfield(data, 'all_data') && ~isempty(data.all_data)
                        [~, h_brs] = contour(data.g.xs{1}, data.g.xs{2}, data.all_data{1,1}, [0 0], 'LineWidth', 2, 'Color', 'b');
                        hold on;
                        [~, h_target] = contour(data.g.xs{1}, data.g.xs{2}, data.data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                        grid on;
                        title('Reachable Set and Target', 'FontSize', 14);
                        
                        if strcmp(vis_model_type, 'doubleInt')
                            xlabel('Position', 'FontSize', 12);
                            ylabel('Velocity', 'FontSize', 12);
                            legend([h_brs, h_target], 'Reachable Set', 'Target Set');
                        else
                            xlabel('Sideslip Angle', 'FontSize', 12);
                            ylabel('Yaw Rate', 'FontSize', 12);
                            legend([h_brs, h_target], 'Reachable Set', 'Target Set');
                        end
                    else
                        text(0.5, 0.5, 'No reachable set data available', 'HorizontalAlignment', 'center');
                        axis off;
                    end
                else
                    % 3D model - show a 2D slice
                    figure;
                    if isfield(data, 'all_data') && ~isempty(data.all_data)
                        % For 3D data, take a slice
                        if ndims(data.all_data{1,1}) == 3
                            % Find middle slice
                            slice_idx = ceil(size(data.all_data{1,1}, 3)/2);
                            
                            % Extract and plot slice
                            slice_data = squeeze(data.all_data{1,1}(:,:,slice_idx));
                            slice_target = squeeze(data.data0(:,:,slice_idx));
                            
                            [~, h_brs] = contour(data.g.xs{2}(:,:,1), data.g.xs{1}(:,:,1), slice_data, [0 0], 'LineWidth', 2, 'Color', 'b');
                            hold on;
                            [~, h_target] = contour(data.g.xs{2}(:,:,1), data.g.xs{1}(:,:,1), slice_target, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                            
                            % Calculate slice value
                            slice_val = data.g.vs{3}(slice_idx);
                            if strcmp(vis_model_type, 'dubinsCar')
                                slice_name = 'Heading';
                                slice_val_deg = slice_val * 180/pi;
                                title(sprintf('Reachable Set and Target (Slice at %s = %.1f°)', slice_name, slice_val_deg), 'FontSize', 14);
                                xlabel('Y Position', 'FontSize', 12);
                                ylabel('X Position', 'FontSize', 12);
                            else
                                slice_name = 'Steering Angle';
                                slice_val_deg = slice_val * 180/pi;
                                title(sprintf('Reachable Set and Target (Slice at %s = %.1f°)', slice_name, slice_val_deg), 'FontSize', 14);
                                xlabel('Sideslip Angle', 'FontSize', 12);
                                ylabel('Yaw Rate', 'FontSize', 12);
                            end
                            
                            legend([h_brs, h_target], 'Reachable Set', 'Target Set');
                            grid on;
                        else
                            text(0.5, 0.5, 'Invalid data dimensions', 'HorizontalAlignment', 'center');
                            axis off;
                        end
                    else
                        text(0.5, 0.5, 'No reachable set data available', 'HorizontalAlignment', 'center');
                        axis off;
                    end
                end
            else
                fprintf('No combined results file found for visualization.\n');
            end
        end
    else
        warning('visualize_reachability_results function not found. Skipping visualization.');
    end
end

fprintf('All processing completed successfully!\n');
if ~isempty(result_folder)
    fprintf('Results saved to: %s\n', result_folder);
end

    profile off;
    % fprintf("Ending profiling!");
    % p = profile('info');
    % profsave(p, sprintf('profile_%s', datestr(now, 'yyyy-mm-dd_HH-MM-SS')))
    % fprintf("Profiling saved.");

end

%% Simple visualization functions for Double Integrator and Dubins Car
function visualize_double_integrator_simple(result_folder)
    % Simple visualization for Double Integrator results
    fprintf('Creating simple Double Integrator visualization...\n');
    
    % Try to load combined results file
    combined_file = fullfile(result_folder, 'brs_combined_results.mat');
    if ~exist(combined_file, 'file')
        combined_file = fullfile(result_folder, 'frs_combined_results.mat');
    end
    
    if exist(combined_file, 'file')
        data = load(combined_file);
        
        figure('Name', 'Double Integrator Reachable Set');
        
        % Plot BRS/FRS boundary
        [~, h_brs] = contour(data.g.xs{1}, data.g.xs{2}, data.all_data{1,1}, [0 0], 'LineWidth', 2, 'Color', 'b');
        hold on;
        
        % Plot target set
        [~, h_target] = contour(data.g.xs{1}, data.g.xs{2}, data.data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
        
        % Add labels and legend
        xlabel('Position', 'FontSize', 12);
        ylabel('Velocity', 'FontSize', 12);
        title('Double Integrator Reachable Set', 'FontSize', 14);
        legend([h_brs, h_target], 'Reachable Set', 'Target Set', 'Location', 'best');
        grid on;
        
        % Add title with additional info
        if isfield(data, 'acceleration_limits')
            acc_limits = data.acceleration_limits;
            title_str = sprintf('Double Integrator BRS (Acceleration Limit: ±%.2f)', acc_limits(1));
            title(title_str, 'FontSize', 14);
        end
    else
        fprintf('No combined results file found for visualization.\n');
    end
end

function visualize_dubins_car_simple(result_folder)
    % Simple visualization for Dubins Car results
    fprintf('Creating simple Dubins Car visualization...\n');
    
    % Try to load combined results file
    combined_file = fullfile(result_folder, 'brs_combined_results.mat');
    if ~exist(combined_file, 'file')
        combined_file = fullfile(result_folder, 'frs_combined_results.mat');
    end
    
    if exist(combined_file, 'file')
        data = load(combined_file);
        
        figure('Name', 'Dubins Car Reachable Set');
        
        % For 3D visualization, show 2D slice at a fixed heading
        % Find middle slice for heading
        slice_idx = ceil(size(data.all_data{1,1}, 3)/2);
        heading_val = data.g.vs{3}(slice_idx) * 180/pi;
        
        % Extract slice data
        brs_slice = squeeze(data.all_data{1,1}(:,:,slice_idx));
        target_slice = squeeze(data.data0(:,:,slice_idx));
        
        % Plot the slice (position space)
        [~, h_brs] = contour(data.g.xs{1}(:,:,1), data.g.xs{2}(:,:,1), brs_slice, [0 0], 'LineWidth', 2, 'Color', 'b');
        hold on;
        [~, h_target] = contour(data.g.xs{1}(:,:,1), data.g.xs{2}(:,:,1), target_slice, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
        
        % Add labels and legend
        xlabel('X Position', 'FontSize', 12);
        ylabel('Y Position', 'FontSize', 12);
        title(sprintf('Dubins Car Reachable Set (Heading = %.1f°)', heading_val), 'FontSize', 14);
        legend([h_brs, h_target], 'Reachable Set', 'Target Set', 'Location', 'best');
        grid on;
        axis equal;
        
        % Add title with additional info
        if isfield(data, 'speed') && isfield(data, 'turning_limits')
            speed = data.speed;
            turn_limit = data.turning_limits(1) * 180/pi;
            title_str = sprintf('Dubins Car (Speed: %.1f m/s, Turn Rate: ±%.1f°/s)', speed, turn_limit);
            title(title_str, 'FontSize', 14);
        end
    else
        fprintf('No combined results file found for visualization.\n');
    end

end
