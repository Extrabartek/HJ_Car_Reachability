function visualize_3d_reachable_sets(result_folder, varargin)
% VISUALIZE_3D_REACHABLE_SETS Creates interactive 3D visualizations of reachable sets
% 
% This function creates a 3D visualization of reachable sets (BRS or FRS) that allows
% the user to interactively explore the reachable set at different time points using
% a slider control.
%
% Inputs:
%   result_folder - Path to the folder containing BRS/FRS computation results
%   varargin      - Optional parameter-value pairs:
%                   'type'       - Type of reachable set ('brs' or 'frs')
%                                 (default: determined from folder name)
%                   'v_idx'      - Index of velocity to visualize (default: 1)
%                   'c_idx'      - Index of control limit to visualize (default: 1)
%                   'saveFigs'   - Whether to save figures (default: false)
%                   'figFormat'  - Format to save figures in (default: 'png')
%                   'isoValue'   - Value for isosurface (default: 0)
%                   'alpha'      - Transparency for the surface (default: 0.5)
%                   'colormap'   - Colormap to use (default: 'jet')
%
% Example:
%   visualize_3d_reachable_sets('steered_brs_results_folder');
%   visualize_3d_reachable_sets('steered_frs_results_folder', 'type', 'frs');

%% Parse inputs
p = inputParser;
p.addRequired('result_folder', @ischar);
p.addParameter('type', '', @ischar);
p.addParameter('v_idx', 1, @isnumeric);
p.addParameter('c_idx', 1, @isnumeric);
p.addParameter('saveFigs', false, @islogical);
p.addParameter('figFormat', 'png', @ischar);
p.addParameter('isoValue', 0, @isnumeric);
p.addParameter('alpha', 0.5, @isnumeric);
p.addParameter('colormap', 'jet', @ischar);

p.parse(result_folder, varargin{:});
opts = p.Results;

%% Check if the folder exists
if ~exist(result_folder, 'dir')
    error('Result folder does not exist: %s', result_folder);
end

%% Determine the type of reachable set (BRS or FRS)
if isempty(opts.type)
    % Try to determine from folder name
    if contains(lower(result_folder), 'brs')
        opts.type = 'brs';
    elseif contains(lower(result_folder), 'frs')
        opts.type = 'frs';
    else
        warning('Could not determine reachable set type from folder name. Defaulting to BRS.');
        opts.type = 'brs';
    end
end

%% Load combined results
combined_file = fullfile(result_folder, [opts.type '_combined_results.mat']);
if ~exist(combined_file, 'file')
    % Try alternative filename formats
    alt_combined_files = {
        fullfile(result_folder, 'brs_combined_results.mat'),
        fullfile(result_folder, 'frs_combined_results.mat')
    };
    
    found = false;
    for i = 1:length(alt_combined_files)
        if exist(alt_combined_files{i}, 'file')
            combined_file = alt_combined_files{i};
            found = true;
            break;
        end
    end
    
    if ~found
        error('Combined results file not found in the specified folder.');
    end
end

disp(['Loading results from: ', combined_file]);

% Load the data
try
    data = load(combined_file);
    
    % Check if this is 3D data (steered model)
    if size(data.g.N, 2) ~= 3 && ndims(data.all_data_full{1, 1}) ~= 4
        error('This function is designed for 3D reachable sets. The data appears to be 2D.');
    end
    
    % Determine parameters
    velocities = data.velocities;
    
    % Determine control limit type and values
    if isfield(data, 'mzmax_values')
        control_limits = data.mzmax_values;
        control_type = 'mzmax';
        control_unit = 'N·m';
    elseif isfield(data, 'dvmax_values')
        control_limits = data.dvmax_values;
        control_type = 'dvmax';
        control_unit = 'rad/s';
    else
        error('Unknown control limit type.');
    end
    
    tau = data.tau;
    g_data = data.g;
    
    % Ensure indices are within valid range
    opts.v_idx = min(max(opts.v_idx, 1), length(velocities));
    opts.c_idx = min(max(opts.c_idx, 1), length(control_limits));
    
    % Get the full time evolution data
    full_data = data.all_data_full{opts.v_idx, opts.c_idx};
    
    % Get target set
    target_data = data.data0;
catch
    error('Error loading data from the results file. Please check the file format.');
end

% Print info about the loaded data
fprintf('Loaded %s data for v = %d m/s, ', upper(opts.type), velocities(opts.v_idx));
if strcmp(control_type, 'dvmax')
    fprintf('dvmax = %.1f°/s\n', control_limits(opts.c_idx) * 180/pi);
else
    fprintf('Mzmax = %d %s\n', control_limits(opts.c_idx), control_unit);
end
fprintf('Time range: %.2f to %.2f seconds\n', tau(1), tau(end));
fprintf('Grid dimensions: %d x %d x %d\n', g_data.N(1), g_data.N(2), g_data.N(3));

%% Create the figure and UI controls
fig = figure('Name', sprintf('3D %s Visualization', upper(opts.type)), ...
    'Position', [100, 100, 1000, 800], ...
    'Color', 'w', ...
    'NumberTitle', 'off', ...
    'MenuBar', 'none', ...
    'Toolbar', 'figure', ...
    'CloseRequestFcn', @onClose);

% Create axes for the 3D plot
ax = axes('Parent', fig, 'Position', [0.1, 0.2, 0.8, 0.7]);

% Create a slider for time selection
slider_time = uicontrol('Style', 'slider', ...
    'Min', 1, 'Max', length(tau), 'Value', length(tau), ...
    'Position', [250, 40, 500, 20], ...
    'Callback', @updatePlot);

% Create a text box to display current time
text_time = uicontrol('Style', 'text', ...
    'Position', [400, 70, 200, 20], ...
    'String', sprintf('Time: %.2f s', tau(end)), ...
    'FontSize', 12);

% Create a checkbox for showing/hiding the target set
check_target = uicontrol('Style', 'checkbox', ...
    'Position', [100, 40, 150, 20], ...
    'String', 'Show Target Set', ...
    'Value', 1, ...
    'Callback', @updatePlot);

% Create a button for resetting the view
button_reset = uicontrol('Style', 'pushbutton', ...
    'Position', [780, 40, 100, 30], ...
    'String', 'Reset View', ...
    'Callback', @resetView);

% Create Play/Pause button
button_play = uicontrol('Style', 'togglebutton', ...
    'Position', [100, 10, 100, 30], ...
    'String', 'Play', ...
    'Value', 0, ...
    'Callback', @togglePlayPause);

% Create speed control slider
uicontrol('Style', 'text', ...
    'Position', [220, 10, 100, 20], ...
    'String', 'Animation Speed:', ...
    'HorizontalAlignment', 'left');

slider_speed = uicontrol('Style', 'slider', ...
    'Min', 0.1, 'Max', 5, 'Value', 1, ...
    'Position', [320, 10, 150, 20]);

% Create a timer for animation
animation_timer = timer('ExecutionMode', 'fixedRate', ...
                         'Period', 0.1, ... 
                         'TimerFcn', @animationUpdate);

% Initialize the plot
time_idx = length(tau);
is_playing = false;
updatePlot();

%% Nested function to update the plot based on slider value
    function updatePlot(~, ~)
        % Get the current time index from the slider
        time_idx = round(get(slider_time, 'Value'));
        
        % Update the time display
        set(text_time, 'String', sprintf('Time: %.2f s', tau(time_idx)));
        
        % Clear the current axes
        cla(ax);
        
        try
            % Get the current data for the selected time
            current_data = full_data(:,:,:,time_idx);
            
            % Create grid vectors for visualization
            x = linspace(g_data.min(1), g_data.max(1), g_data.N(1));
            y = linspace(g_data.min(2), g_data.max(2), g_data.N(2));
            z = linspace(g_data.min(3), g_data.max(3), g_data.N(3));
            
            % Create the 3D grid for visualization
            [X, Y, Z] = meshgrid(x, y, z);
            
            % Transpose current_data to match meshgrid format if needed
            if size(current_data, 1) == length(x) && size(current_data, 2) == length(y) && size(current_data, 3) == length(z)
                current_data = permute(current_data, [2 1 3]);
            end
            
            % Create isosurface for the reachable set
            iso_value = opts.isoValue;
            p = patch(isosurface(X, Y, Z, current_data, iso_value));
            
            % Set properties
            set(p, 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', opts.alpha);
            
            % Add lighting effects
            lighting gouraud
            camlight
            
            % Show target set if requested
            if get(check_target, 'Value') == 1
                % Transpose target_data to match meshgrid format if needed
                if size(target_data, 1) == length(x) && size(target_data, 2) == length(y) && size(target_data, 3) == length(z)
                    target_data_vis = permute(target_data, [2 1 3]);
                else
                    target_data_vis = target_data;
                end
                
                % Create isosurface for the target set
                t = patch(isosurface(X, Y, Z, target_data_vis, iso_value));
                set(t, 'FaceColor', 'g', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
            end
            
            % Set axis properties
            xlabel(['Yaw Rate (deg/s)'], 'FontSize', 12);
            ylabel(['Sideslip Angle (deg)'], 'FontSize', 12);
            zlabel(['Steering Angle (deg)'], 'FontSize', 12);
            
            % Convert tick labels from radians to degrees
            xticks_rad = get(ax, 'XTick');
            yticks_rad = get(ax, 'YTick');
            zticks_rad = get(ax, 'ZTick');
            
            set(ax, 'XTickLabel', arrayfun(@(x) sprintf('%.1f', x*180/pi), xticks_rad, 'UniformOutput', false));
            set(ax, 'YTickLabel', arrayfun(@(y) sprintf('%.1f', y*180/pi), yticks_rad, 'UniformOutput', false));
            set(ax, 'ZTickLabel', arrayfun(@(z) sprintf('%.1f', z*180/pi), zticks_rad, 'UniformOutput', false));
            
            % Set title
            if strcmp(control_type, 'dvmax')
                control_str = sprintf('Max Steering Rate = %.1f°/s', control_limits(opts.c_idx) * 180/pi);
            else
                control_str = sprintf('Mzmax = %d %s', control_limits(opts.c_idx), control_unit);
            end
            
            title(sprintf('3D %s at t = %.2f s (v = %d m/s, %s)', ...
                upper(opts.type), tau(time_idx), velocities(opts.v_idx), control_str), ...
                'FontSize', 14, 'FontWeight', 'bold');
            
            % Add grid and box
            grid on;
            box on;
            
            % Set colormap
            colormap(ax, opts.colormap);

            % Set axis limits
            xlim([min(x) max(x)]);
            ylim([min(y) max(y)]);
            zlim([min(z) max(z)]);
            
        catch err
            % Display error message in the plot area
            text(0.5, 0.5, sprintf('Error visualizing 3D data:\n%s', err.message), ...
                'Parent', ax, 'HorizontalAlignment', 'center', 'FontSize', 12, 'Color', 'r');
            
            % Reset axis limits
            axis(ax, [0 1 0 1]);
            
            % Log the error to console
            fprintf('Visualization error: %s\n', err.message);
        end
            
        % Save figure if requested
        if opts.saveFigs
            fig_folder = fullfile(result_folder, 'figures_3d');
            if ~exist(fig_folder, 'dir')
                mkdir(fig_folder);
            end
            
            fig_filename = sprintf('3d_%s_v%d_%s%d_t%.2f.%s', ...
                opts.type, velocities(opts.v_idx), control_type, control_limits(opts.c_idx), ...
                tau(time_idx), opts.figFormat);
            
            saveas(fig, fullfile(fig_folder, fig_filename));
            fprintf('Saved 3D visualization to %s\n', fullfile(fig_folder, fig_filename));
        end
    end

%% Nested function to reset the view
    function resetView(~, ~)
        view(ax, 3); % Reset to default 3D view
    end

% Display help message
disp('Interactive controls:');
disp('  - Use the slider to change the visualization time');
disp('  - Check/uncheck the box to show/hide the target set');
disp('  - Click "Reset View" to reset the 3D camera view');
disp('  - Use the normal MATLAB figure toolbar for rotation, zoom, etc.');

%% Nested function to toggle Play/Pause
    function togglePlayPause(src, ~)
        is_playing = get(src, 'Value');
        if is_playing
            % Start playing - change button text and start timer
            set(src, 'String', 'Pause');
            % Set timer period based on speed slider
            animation_timer.Period = 1/get(slider_speed, 'Value');
            start(animation_timer);
        else
            % Stop playing - change button text and stop timer
            set(src, 'String', 'Play');
            stop(animation_timer);
        end
    end

%% Nested function for animation timer update
    function animationUpdate(~, ~)
        % Get current time index
        current_idx = get(slider_time, 'Value');
        
        % Move to next time step, loop back to beginning if at end
        if current_idx >= length(tau)
            % Reset to beginning
            next_idx = 1;
        else
            next_idx = current_idx + 1;
        end
        
        % Update slider and plot
        set(slider_time, 'Value', next_idx);
        updatePlot();
        
        % Update timer period in case speed was changed
        animation_timer.Period = 1/get(slider_speed, 'Value');
    end

%% Handle figure close
    function onClose(~, ~)
        % Stop timer when figure is closed
        if strcmp(animation_timer.Running, 'on')
            stop(animation_timer);
        end
        delete(animation_timer);
        delete(fig);
    end 

end