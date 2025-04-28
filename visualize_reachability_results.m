function visualize_reachability_results(result_folder, varargin)
% VISUALIZE_REACHABILITY_RESULTS Unified visualization for reachability results
%
% This function provides a unified interface for visualizing both BRS and FRS results
% for different vehicle models (standard bicycle and steered bicycle).
%
% Inputs:
%   result_folder - Path to the folder containing computation results
%   varargin      - Optional parameter-value pairs:
%                   'plotType'   - Types of plots to generate (cell array)
%                                 Options (2D model): 'control', 'detailed', 'comparison', 
%                                                    'velocity_stack', 'derivative', 'tire'
%                                 Options (3D model): 'slices', 'detailed', 'comparison'
%                                 (default: auto-detect based on model)
%                   'saveFigs'   - Whether to save figures (default: true)
%                   'figFormat'  - Format to save figures in (default: 'png')
%                   'deltaSlices'- Specific steering angle slices to visualize for 3D model
%                                 (default: [-0.1, 0, 0.1] rad)
%                   'controlIdx' - Control limit index to visualize (default: 1)
%                   'velocityIdx'- Velocity index to visualize (default: 1)
%                   'isoValue'   - Value for isosurface in 3D plots (default: 0)
%                   'opacity'    - Opacity for BRS/FRS surfaces (default: 0.3)
%
% Examples:
%   % Basic usage - auto-detects model type and plot types:
%   visualize_reachability_results('path/to/results/folder');
%
%   % Specific plot types for 2D model:
%   visualize_reachability_results('path/to/results', 'plotType', {'control', 'detailed'});
%
%   % Specific plot types for 3D model:
%   visualize_reachability_results('path/to/results', 'plotType', {'slices'}, 
%                                 'deltaSlices', [-0.05, 0, 0.05]);

%% Parse inputs
p = inputParser;
p.addRequired('result_folder', @ischar);
p.addParameter('plotType', {}, @iscell);
p.addParameter('saveFigs', true, @islogical);
p.addParameter('figFormat', 'png', @ischar);
p.addParameter('deltaSlices', [-0.1, 0, 0.1], @isnumeric);
p.addParameter('controlIdx', 1, @isnumeric);
p.addParameter('velocityIdx', 1, @isnumeric);
p.addParameter('isoValue', 0, @isnumeric);
p.addParameter('opacity', 0.3, @(x) isnumeric(x) && x >= 0 && x <= 1);

p.parse(result_folder, varargin{:});
opts = p.Results;

%% Check if the folder exists
if ~exist(result_folder, 'dir')
    error('Result folder does not exist: %s', result_folder);
end

%% Detect result type (BRS/FRS and model type)
% First check if it's BRS or FRS by looking for result files
brs_combined_file = fullfile(result_folder, 'brs_combined_results.mat');
frs_combined_file = fullfile(result_folder, 'frs_combined_results.mat');

if exist(brs_combined_file, 'file')
    computation_type = 'brs';
    result_file = brs_combined_file;
elseif exist(frs_combined_file, 'file')
    computation_type = 'frs';
    result_file = frs_combined_file;
else
    error('Could not find combined results file in the specified folder');
end

disp(['Loading ' upper(computation_type) ' data from: ', result_file]);
result_data = load(result_file);

%% Detect model type (2D standard bicycle or 3D steered bicycle)
if isfield(result_data, 'g') && isfield(result_data.g, 'N')
    grid_dims = length(result_data.g.N);
    if grid_dims == 2
        model_type = '2d';
        disp('Detected 2D model (standard bicycle with yaw moment control)');
    elseif grid_dims == 3
        model_type = '3d';
        disp('Detected 3D model (steered bicycle)');
    else
        error('Unsupported model with %d dimensions', grid_dims);
    end
else
    error('Could not determine model type from result data');
end

%% Extract data based on detected model and computation type
% Extract grid
g = result_data.g;

% Extract target set
data0 = result_data.data0;

% Extract velocities
velocities = result_data.velocities;

% Extract control limits based on model type
if strcmp(model_type, '2d')
    control_limits = result_data.mzmax_values;
    control_type = 'mz';
    control_unit = 'N·m';
else
    control_limits = result_data.dvmax_values;
    control_type = 'dv';
    control_unit = 'rad/s';
end

% Extract time vector
tau = result_data.tau;

% Extract BRS/FRS data
all_data = result_data.all_data;
if isfield(result_data, 'all_data_full')
    all_data_full = result_data.all_data_full;
else
    all_data_full = [];
end

% Extract control data if available
if isfield(result_data, 'control_data')
    control_data = result_data.control_data;
else
    control_data = [];
end

%% Validate and adjust indices
if opts.velocityIdx > length(velocities)
    warning('velocityIdx (%d) exceeds available velocities. Using index 1.', opts.velocityIdx);
    opts.velocityIdx = 1;
end

if opts.controlIdx > length(control_limits)
    warning('controlIdx (%d) exceeds available control limits. Using index 1.', opts.controlIdx);
    opts.controlIdx = 1;
end

%% Set default plot types if not provided
if isempty(opts.plotType)
    if strcmp(model_type, '2d')
        opts.plotType = {'control', 'detailed', 'comparison'};
    else
        opts.plotType = {'slices', 'detailed', 'comparison'};
    end
    
    fprintf('Using default plot types: %s\n', strjoin(opts.plotType, ', '));
end

%% Create output folder for figures if saving is enabled
if opts.saveFigs
    fig_folder = fullfile(result_folder, 'figures');
    if ~exist(fig_folder, 'dir')
        mkdir(fig_folder);
        disp(['Created figures directory: ', fig_folder]);
    end
end

%% Setup visualization parameters
% Create a fixed simple red/blue colormap for clear distinction between positive and negative control
custom_cmap = [
    0.7 0 0;    % Dark red for strong negative control
    1 0.5 0.5;  % Light red for weak negative control
    0.5 0.5 1;  % Light blue for weak positive control
    0 0 0.7     % Dark blue for strong positive control
];

%% Process each plot type
for i = 1:length(opts.plotType)
    current_plot_type = opts.plotType{i};
    
    % Standardize plot type to lowercase
    current_plot_type = lower(current_plot_type);
    
    % Generate appropriate visualization based on model type and plot type
    if strcmp(model_type, '2d')
        % 2D model visualizations
        switch current_plot_type
            case 'control'
                generate_2d_control_plots();
                
            case 'detailed'
                generate_2d_detailed_plots();
                
            case 'comparison'
                generate_2d_comparison_plots();
                
            case 'velocity_stack'
                generate_3d_velocity_stack();
                
            case 'derivative'
                generate_2d_derivative_plots();
                
            case 'tire'
                generate_tire_curves();
                
            otherwise
                warning('Unknown plot type for 2D model: %s', current_plot_type);
        end
    else
        % 3D model visualizations
        switch current_plot_type
            case 'slices'
                generate_3d_slice_plots();
                
            case 'detailed'
                generate_3d_detailed_plots();
                
            case 'comparison'
                generate_3d_comparison_plots();
                
            otherwise
                warning('Unknown plot type for 3D model: %s', current_plot_type);
        end
    end
end

disp('Visualization complete!');

%% Nested function to generate 2D control plots (for standard bicycle model)
    function generate_2d_control_plots()
        disp('Generating control plots for 2D model...');
        
        % Get indices to visualize
        v_idx = opts.velocityIdx;
        m_idx = opts.controlIdx;
        
        % Check if indices are valid
        if v_idx > length(velocities) || m_idx > length(control_limits)
            warning('Invalid indices for visualization. Using defaults.');
            v_idx = 1;
            m_idx = 1;
        end
        
        figure('Name', sprintf('Control - v=%d, %s=%d', velocities(v_idx), control_type, control_limits(m_idx)));
        clf;
        
        % Create a larger figure
        set(gcf, 'Position', [100, 100, 800, 600]);
        
        % Convert grid values from radians to degrees for plotting
        xs1_deg = g.xs{2} * 180/pi;  % Convert sideslip angle to degrees
        xs2_deg = g.xs{1} * 180/pi;  % Convert yaw rate to degrees
        
        % Get the BRS/FRS data and control data for this combination
        current_data = all_data{v_idx, m_idx};
        
        if ~isempty(control_data)
            current_control = control_data{v_idx, m_idx};
            
            % Plot control input with colored background
            control_plot = pcolor(xs1_deg, xs2_deg, current_control);
            control_plot.EdgeColor = 'none';
            colormap(custom_cmap);
            
            % Add a colorbar
            cb = colorbar;
            if strcmp(control_type, 'mz')
                title(cb, 'Yaw Moment (N·m)');
                
                % Set the colorbar limits to match the Mzmax values
                caxis([-control_limits(m_idx), control_limits(m_idx)]);
            else
                title(cb, 'Steering Rate (rad/s)');
                
                % Set the colorbar limits to match the dvmax values
                caxis([-control_limits(m_idx), control_limits(m_idx)]);
            end
        else
            % If control data is not available, just show the BRS/FRS
            warning('Control data not available. Showing only %s boundary.', upper(computation_type));
        end
        
        hold on;
        
        % Plot the BRS/FRS boundary
        [~, h_data] = contour(xs1_deg, xs2_deg, current_data, [0 0], 'LineWidth', 2, 'Color', 'k');
        
        % Plot the target set
        [~, h_target] = contour(xs1_deg, xs2_deg, data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
        
        % Add grid, labels, and legend
        grid on;
        xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
        ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
        title(sprintf('%s and Optimal Control for v = %d m/s, %s = %d %s', ...
              upper(computation_type), velocities(v_idx), upper(control_type), control_limits(m_idx), control_unit), ...
              'FontSize', 14, 'FontWeight', 'bold');
              
        % Set up legend based on what's plotted
        if ~isempty(control_data)
            legend([h_data, h_target], [upper(computation_type) ' Boundary'], 'Target Set', 'Location', 'best', 'FontSize', 10);
        else
            legend(h_data, [upper(computation_type) ' Boundary'], 'Location', 'best', 'FontSize', 10);
        end
        
        % Add a better grid
        grid minor;
        
        % Calculate axis limits
        x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
        y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
        
        % Set axis limits
        xlim(x_limits);
        ylim(y_limits);
        
        % Save figure if enabled
        if opts.saveFigs
            if strcmp(control_type, 'mz')
                fig_filename = sprintf('control_v%d_mz%d.%s', ...
                    velocities(v_idx), control_limits(m_idx), opts.figFormat);
            else
                fig_filename = sprintf('control_v%d_dv%.0f.%s', ...
                    velocities(v_idx), control_limits(m_idx)*180/pi, opts.figFormat);
            end
            saveas(gcf, fullfile(fig_folder, fig_filename));
            fprintf('Saved control plot to %s\n', fullfile(fig_folder, fig_filename));
        end
    end

%% Nested function to generate 2D detailed plots
    function generate_2d_detailed_plots()
        disp('Generating detailed analysis plots for 2D model...');
        
        % Get indices to visualize
        v_idx = opts.velocityIdx;
        m_idx = opts.controlIdx;
        
        % Check if indices are valid
        if v_idx > length(velocities) || m_idx > length(control_limits)
            warning('Invalid indices for visualization. Using defaults.');
            v_idx = 1;
            m_idx = 1;
        end
        
        figure('Name', sprintf('Detailed - v=%d, %s=%d', velocities(v_idx), control_type, control_limits(m_idx)));
        clf;
        
        % Create a larger figure
        set(gcf, 'Position', [100, 100, 900, 700]);
        
        % Convert grid values from radians to degrees for plotting
        xs1_deg = g.xs{2} * 180/pi;  % Convert sideslip angle to degrees
        xs2_deg = g.xs{1} * 180/pi;  % Convert yaw rate to degrees
        
        % Calculate axis limits
        x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
        y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
        
        % Get the data for this combination
        current_data = all_data{v_idx, m_idx};
        
        % 1. Control Input Visualization (if available)
        subplot(2, 2, 1);
        
        if ~isempty(control_data)
            current_control = control_data{v_idx, m_idx};
            
            % Create a pcolor plot with the custom colormap
            control_plot = pcolor(xs1_deg, xs2_deg, current_control);
            control_plot.EdgeColor = 'none';
            colormap(gca, custom_cmap);
            
            if strcmp(control_type, 'mz')
                caxis([-control_limits(m_idx), control_limits(m_idx)]);
                
                % Add colorbar and labels
                cb = colorbar;
                title(cb, 'Yaw Moment (N·m)');
            else
                caxis([-control_limits(m_idx), control_limits(m_idx)]);
                
                % Add colorbar and labels
                cb = colorbar;
                title(cb, 'Steering Rate (rad/s)');
            end
            
            % Add contour of BRS/FRS boundary
            hold on;
            [~, h_data] = contour(xs1_deg, xs2_deg, current_data, [0 0], 'LineWidth', 2, 'Color', 'k');
            
            title('Optimal Control Input', 'FontSize', 12);
        else
            % If control data is not available, show blank panel with message
            text(0.5, 0.5, 'Control data not available', 'HorizontalAlignment', 'center', 'FontSize', 12);
            axis off;
        end
        
        xlabel('Sideslip Angle (deg)', 'FontSize', 10);
        ylabel('Yaw Rate (deg/s)', 'FontSize', 10);
        xlim(x_limits);
        ylim(y_limits);
        grid on;
        
        % 2. BRS/FRS Boundary with Control Switching Surface
        subplot(2, 2, 2);
        
        % Plot BRS/FRS boundary
        [~, h_data] = contour(xs1_deg, xs2_deg, current_data, [0 0], 'LineWidth', 2, 'Color', 'k');
        hold on;
        
        % Plot target set
        [~, h_target] = contour(xs1_deg, xs2_deg, data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
        
        % Find and plot the control switching surface if we have control data
        if ~isempty(control_data)
            % Control switching occurs when the sign of the gradient of value function changes
            derivs = extractCostates(g, current_data);
            
            if strcmp(control_type, 'mz')
                switching_condition = derivs{2};  % For yaw moment control, the derivative w.r.t. gamma is key
            else
                switching_condition = derivs{3};  % For steering control, the derivative w.r.t. delta is key
            end
            
            [~, h_switch] = contour(xs1_deg, xs2_deg, switching_condition, [0 0], 'LineWidth', 1.5, 'Color', 'r', 'LineStyle', '-.'); 
            
            title([upper(computation_type) ' Boundary and Control Switching Surface'], 'FontSize', 12);
            legend([h_data, h_target, h_switch], [upper(computation_type) ' Boundary'], 'Target Set', 'Control Switching', 'Location', 'best', 'FontSize', 8);
        else
            title([upper(computation_type) ' Boundary'], 'FontSize', 12);
            legend([h_data, h_target], [upper(computation_type) ' Boundary'], 'Target Set', 'Location', 'best', 'FontSize', 8);
        end
        
        xlabel('Sideslip Angle (deg)', 'FontSize', 10);
        ylabel('Yaw Rate (deg/s)', 'FontSize', 10);
        xlim(x_limits);
        ylim(y_limits);
        grid on;
        
        % 3. Value Function Surface
        subplot(2, 2, 3:4);
        
        % Plot the value function as a surface
        surf(xs1_deg, xs2_deg, current_data, 'EdgeColor', 'none');
        colormap(gca, parula);
        
        % Adjust view angle
        view([-30, 30]);
        
        % Add the zero level set (BRS/FRS boundary)
        hold on;
        [~, h_data] = contour3(xs1_deg, xs2_deg, current_data, [0 0], 'LineWidth', 2, 'Color', 'r');
        
        % Add colorbar and labels
        cb = colorbar;
        title(cb, 'Value Function');
        
        title('Value Function Surface', 'FontSize', 12);
        xlabel('Sideslip Angle (deg)', 'FontSize', 10);
        ylabel('Yaw Rate (deg/s)', 'FontSize', 10);
        zlabel('Value', 'FontSize', 10);
        grid on;
        
        % Add a main title for the entire figure
        if strcmp(control_type, 'mz')
            sgtitle(sprintf('Detailed Analysis (v = %d m/s, Mzmax = %d N·m)', ...
                    velocities(v_idx), control_limits(m_idx)), 'FontSize', 14, 'FontWeight', 'bold');
        else
            sgtitle(sprintf('Detailed Analysis (v = %d m/s, dvmax = %.1f deg/s)', ...
                    velocities(v_idx), control_limits(m_idx)*180/pi), 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        % Save figure if enabled
        if opts.saveFigs
            if strcmp(control_type, 'mz')
                fig_filename = sprintf('detailed_v%d_mz%d.%s', ...
                    velocities(v_idx), control_limits(m_idx), opts.figFormat);
            else
                fig_filename = sprintf('detailed_v%d_dv%.0f.%s', ...
                    velocities(v_idx), control_limits(m_idx)*180/pi, opts.figFormat);
            end
            saveas(gcf, fullfile(fig_folder, fig_filename));
            fprintf('Saved detailed plot to %s\n', fullfile(fig_folder, fig_filename));
        end
    end

%% Nested function to generate 2D comparison plots
    function generate_2d_comparison_plots()
        disp('Generating comparison plots for 2D model...');
        
        % Comparison of different velocities (if applicable)
        if length(velocities) > 1
            % Compare for different velocities with the same control limit
            m_idx = opts.controlIdx;
            
            if m_idx > length(control_limits)
                warning('Invalid control index. Using first control limit.');
                m_idx = 1;
            end
            
            figure('Name', sprintf('Velocity Comparison - %s=%d', control_type, control_limits(m_idx)));
            clf;
            
            % Create a larger figure
            set(gcf, 'Position', [100, 100, 800, 600]);
            
            % Convert grid values from radians to degrees for plotting
            xs1_deg = g.xs{2} * 180/pi;
            xs2_deg = g.xs{1} * 180/pi;
            
            % Calculate axis limits
            x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
            y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
            
            % Plot target set
            [~, h_target] = contour(xs1_deg, xs2_deg, data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
            hold on;
            
            % Create color map for different velocities
            vel_colors = jet(length(velocities));
            
            % Plot boundary for each velocity
            h_data = zeros(length(velocities), 1);
            for v_idx = 1:length(velocities)
                [~, h_data(v_idx)] = contour(xs1_deg, xs2_deg, all_data{v_idx, m_idx}, [0 0], ...
                    'LineWidth', 2, 'Color', vel_colors(v_idx,:));
            end
            
            % Create legend
            legend_entries = cell(length(velocities) + 1, 1);
            for v_idx = 1:length(velocities)
                legend_entries{v_idx} = sprintf('v = %d m/s', velocities(v_idx));
            end
            legend_entries{end} = 'Target Set';
            
            % Add labels, title and legend
            grid on;
            xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
            ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
            
            if strcmp(control_type, 'mz')
                title(sprintf('%s Comparison for Different Velocities (Mzmax = %d N·m)', ...
                      upper(computation_type), control_limits(m_idx)), 'FontSize', 14, 'FontWeight', 'bold');
            else
                title(sprintf('%s Comparison for Different Velocities (dvmax = %.1f deg/s)', ...
                      upper(computation_type), control_limits(m_idx)*180/pi), 'FontSize', 14, 'FontWeight', 'bold');
            end
            
            legend([h_data; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
            
            % Add a better grid
            grid minor;
            
            % Set axis limits
            xlim(x_limits);
            ylim(y_limits);
            
            % Save figure if enabled
            if opts.saveFigs
                if strcmp(control_type, 'mz')
                    fig_filename = sprintf('velocity_comparison_mz%d.%s', ...
                        control_limits(m_idx), opts.figFormat);
                else
                    fig_filename = sprintf('velocity_comparison_dv%.0f.%s', ...
                        control_limits(m_idx)*180/pi, opts.figFormat);
                end
                saveas(gcf, fullfile(fig_folder, fig_filename));
                fprintf('Saved velocity comparison plot to %s\n', fullfile(fig_folder, fig_filename));
            end
        end
        
        % Comparison of different control limits (if applicable)
        if length(control_limits) > 1
            % Compare for different control limits with the same velocity
            v_idx = opts.velocityIdx;
            
            if v_idx > length(velocities)
                warning('Invalid velocity index. Using first velocity.');
                v_idx = 1;
            end
            
            figure('Name', sprintf('%s Comparison - v=%d', control_type, velocities(v_idx)));
            clf;
            
            % Create a larger figure
            set(gcf, 'Position', [100, 100, 800, 600]);
            
            % Convert grid values from radians to degrees for plotting
            xs1_deg = g.xs{2} * 180/pi;
            xs2_deg = g.xs{1} * 180/pi;
            
            % Calculate axis limits
            x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
            y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
            
            % Plot target set
            [~, h_target] = contour(xs1_deg, xs2_deg, data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
            hold on;
            
            % Create color map for different control limits
            control_colors = jet(length(control_limits));
            
            % Plot boundary for each control limit
            h_data = zeros(length(control_limits), 1);
            for m_idx = 1:length(control_limits)
                [~, h_data(m_idx)] = contour(xs1_deg, xs2_deg, all_data{v_idx, m_idx}, [0 0], ...
                    'LineWidth', 2, 'Color', control_colors(m_idx,:));
            end
            
            % Create legend
            legend_entries = cell(length(control_limits) + 1, 1);
            for m_idx = 1:length(control_limits)
                if strcmp(control_type, 'mz')
                    legend_entries{m_idx} = sprintf('Mzmax = %d N·m', control_limits(m_idx));
                else
                    legend_entries{m_idx} = sprintf('dvmax = %.1f deg/s', control_limits(m_idx)*180/pi);
                end
            end
            legend_entries{end} = 'Target Set';
            
            % Add labels, title and legend
            grid on;
            xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
            ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
            
            if strcmp(control_type, 'mz')
                title(sprintf('%s Comparison for Different Yaw Moment Limits (v = %d m/s)', ...
                      upper(computation_type), velocities(v_idx)), 'FontSize', 14, 'FontWeight', 'bold');
            else
                title(sprintf('%s Comparison for Different Steering Rate Limits (v = %d m/s)', ...
                      upper(computation_type), velocities(v_idx)), 'FontSize', 14, 'FontWeight', 'bold');
            end
            
            legend([h_data; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
            
            % Add a better grid
            grid minor;
            
            % Set axis limits
            xlim(x_limits);
            ylim(y_limits);
            
            % Save figure if enabled
            if opts.saveFigs
                fig_filename = sprintf('control_comparison_v%d.%s', ...
                    velocities(v_idx), opts.figFormat);
                saveas(gcf, fullfile(fig_folder, fig_filename));
                fprintf('Saved control comparison plot to %s\n', fullfile(fig_folder, fig_filename));
            end
        end
        
        % Time evolution visualization (if full data is available)
        if ~isempty(all_data_full)
            v_idx = opts.velocityIdx;
            m_idx = opts.controlIdx;
            
            if v_idx > length(velocities) || m_idx > length(control_limits)
                warning('Invalid indices for time evolution. Using defaults.');
                v_idx = 1;
                m_idx = 1;
            end
            
            if ~isempty(all_data_full{v_idx, m_idx})
                figure('Name', [upper(computation_type) ' Time Evolution']);
                clf;
                
                % Create a larger figure
                set(gcf, 'Position', [100, 100, 800, 600]);
                
                % Convert grid values from radians to degrees for plotting
                xs1_deg = g.xs{2} * 180/pi;
                xs2_deg = g.xs{1} * 180/pi;
                
                % Calculate axis limits
                x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
                y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
                
                % Get full time evolution data
                full_data = all_data_full{v_idx, m_idx};
                
                hold on;
                
                % Select a subset of time points to visualize (to avoid clutter)
                num_time_points = size(full_data, 3);
                if num_time_points > 6
                    time_indices = round(linspace(2, num_time_points, 5));
                else
                    time_indices = 2:num_time_points;
                end
                
                % Create color map for different time points
                time_colors = winter(length(time_indices));
                
                % Plot boundary for each time point
                h_time = zeros(length(time_indices), 1);
                for t_idx = 1:length(time_indices)
                    time_point = time_indices(t_idx);
                    [~, h_time(t_idx)] = contour(xs1_deg, xs2_deg, full_data(:,:,time_point), [0 0], ...
                        'LineWidth', 2, 'Color', time_colors(t_idx,:));
                end
                
                % Create legend
                legend_entries = cell(length(time_indices) + 1, 1);
                for t_idx = 1:length(time_indices)
                    legend_entries{t_idx} = sprintf('t = %.2f s', tau(time_indices(t_idx)));
                end
                legend_entries{end} = 'Target Set';
    
                % Plot target set
                [~, h_target] = contour(xs1_deg, xs2_deg, data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
    
                % Add labels, title and legend
                grid on;
                xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
                ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
                
                if strcmp(control_type, 'mz')
                    title(sprintf('%s Time Evolution (v = %d m/s, Mzmax = %d N·m)', ...
                          upper(computation_type), velocities(v_idx), control_limits(m_idx)), ...
                          'FontSize', 14, 'FontWeight', 'bold');
                else
                    title(sprintf('%s Time Evolution (v = %d m/s, dvmax = %.1f deg/s)', ...
                          upper(computation_type), velocities(v_idx), control_limits(m_idx)*180/pi), ...
                          'FontSize', 14, 'FontWeight', 'bold');
                end
                
                legend([h_time; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
                
                % Add a better grid
                grid minor;
                
                % Set axis limits
                xlim(x_limits);
                ylim(y_limits);
                
                % Save figure if enabled
                if opts.saveFigs
                    if strcmp(control_type, 'mz')
                        fig_filename = sprintf('time_evolution_v%d_mz%d.%s', ...
                            velocities(v_idx), control_limits(m_idx), opts.figFormat);
                    else
                        fig_filename = sprintf('time_evolution_v%d_dv%.0f.%s', ...
                            velocities(v_idx), control_limits(m_idx)*180/pi, opts.figFormat);
                    end
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                    fprintf('Saved time evolution plot to %s\n', fullfile(fig_folder, fig_filename));
                end
            else
                warning('Full time evolution data not available for selected indices.');
            end
        end
    end

%% Nested function to generate 3D stacked velocity visualization
    function generate_3d_velocity_stack()
        disp('Generating 3D velocity stack visualization...');
        
        % This visualization only makes sense if we have multiple velocities
        if length(velocities) <= 1
            warning('Need multiple velocities for 3D velocity stack. Skipping.');
            return;
        end
        
        % Get control index
        m_idx = opts.controlIdx;
        if m_idx > length(control_limits)
            warning('Invalid control index. Using first control limit.');
            m_idx = 1;
        end
        
        figure('Name', sprintf('Velocity Stack - %s=%d', control_type, control_limits(m_idx)));
        clf;
        
        % Create a larger figure
        set(gcf, 'Position', [100, 100, 800, 600]);
        
        % Convert grid values from radians to degrees for plotting
        xs1_deg = g.xs{2} * 180/pi;
        xs2_deg = g.xs{1} * 180/pi;
        
        % Calculate axis limits
        x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
        y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
        
        hold on;
        
        % Create color map for different velocities
        vel_colors = jet(length(velocities));
        
        % Plot boundary for each velocity as a 3D contour
        h_data = zeros(length(velocities), 1);
        for v_idx = 1:length(velocities)
            [~, h_data(v_idx)] = contour3(xs1_deg, xs2_deg, ...
                all_data{v_idx, m_idx} + velocities(v_idx), ...
                [velocities(v_idx) velocities(v_idx)], ...
                'LineWidth', 2, 'Color', vel_colors(v_idx,:));
            
            % Also plot target set at this velocity level
            contour3(xs1_deg, xs2_deg, data0 + velocities(v_idx), ...
                [velocities(v_idx) velocities(v_idx)], ...
                'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
        end
        
        % Create legend
        legend_entries = cell(length(velocities) + 1, 1);
        for v_idx = 1:length(velocities)
            legend_entries{v_idx} = sprintf('v = %d m/s', velocities(v_idx));
        end
        legend_entries{end} = 'Target Set';
        
        % Add labels, title and view settings
        grid on;
        xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
        ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
        zlabel('Longitudinal Velocity (m/s)', 'FontSize', 12);
        
        if strcmp(control_type, 'mz')
            title(sprintf('%s Comparison at Different Velocities (Mzmax = %d N·m)', ...
                  upper(computation_type), control_limits(m_idx)), 'FontSize', 14, 'FontWeight', 'bold');
        else
            title(sprintf('%s Comparison at Different Velocities (dvmax = %.1f deg/s)', ...
                  upper(computation_type), control_limits(m_idx)*180/pi), 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        % Add a better grid
        grid minor;
        
        % Set axis limits
        xlim(x_limits);
        ylim(y_limits);
        zlim([0, max(velocities) + 10]);
        
        % Set view angle
        view([-30, 30]);
        
        % Save figure if enabled
        if opts.saveFigs
            if strcmp(control_type, 'mz')
                fig_filename = sprintf('velocity_stack_mz%d.%s', ...
                    control_limits(m_idx), opts.figFormat);
            else
                fig_filename = sprintf('velocity_stack_dv%.0f.%s', ...
                    control_limits(m_idx)*180/pi, opts.figFormat);
            end
            saveas(gcf, fullfile(fig_folder, fig_filename));
            fprintf('Saved velocity stack plot to %s\n', fullfile(fig_folder, fig_filename));
        end
    end

%% Nested function to generate 2D derivative plots
    function generate_2d_derivative_plots()
        disp('Generating derivative plots for 2D model...');
        
        % Get indices to visualize
        v_idx = opts.velocityIdx;
        m_idx = opts.controlIdx;
        
        % Check if indices are valid
        if v_idx > length(velocities) || m_idx > length(control_limits)
            warning('Invalid indices for visualization. Using defaults.');
            v_idx = 1;
            m_idx = 1;
        end
        
        figure('Name', sprintf('Derivatives - v=%d, %s=%d', velocities(v_idx), control_type, control_limits(m_idx)));
        clf;
        
        % Create a larger figure
        set(gcf, 'Position', [100, 100, 1000, 800]);
        
        % Extract data for the current setup
        current_data = all_data{v_idx, m_idx};
        
        % Convert grid values from radians to degrees for plotting
        xs1_deg = g.xs{2} * 180/pi;  % Convert sideslip angle to degrees
        xs2_deg = g.xs{1} * 180/pi;  % Convert yaw rate to degrees
        
        % Compute gradients of the value function
        derivs = computeGradients(g, current_data);
        
        % Extract the derivatives with respect to each state
        gamma_derivative = derivs{1};  % dV/d(yaw rate)
        beta_derivative = derivs{2};   % dV/d(sideslip)
        
        % 1. Plot derivative with respect to sideslip angle (beta)
        subplot(2, 2, 1);
        beta_deriv_plot = pcolor(xs1_deg, xs2_deg, beta_derivative);
        beta_deriv_plot.EdgeColor = 'none';
        colormap(gca, 'parula');
        cb = colorbar;
        title(cb, 'dV/dβ');
        hold on;
        
        % Add BRS/FRS boundary
        [~, h_data] = contour(xs1_deg, xs2_deg, current_data, [0 0], 'LineWidth', 2, 'Color', 'k');
        
        % Add zero-crossing contour for the derivative
        [~, h_zero] = contour(xs1_deg, xs2_deg, beta_derivative, [0 0], 'LineWidth', 1.5, 'Color', 'r', 'LineStyle', '--');
        
        title('Derivative w.r.t. Sideslip Angle (β)', 'FontSize', 12);
        xlabel('Sideslip Angle (degrees)', 'FontSize', 10);
        ylabel('Yaw Rate (degrees/s)', 'FontSize', 10);
        legend([h_data, h_zero], [upper(computation_type) ' Boundary'], 'dV/dβ = 0', 'Location', 'best');
        grid on;
        
        % 2. Plot derivative with respect to yaw rate (gamma)
        subplot(2, 2, 2);
        gamma_deriv_plot = pcolor(xs1_deg, xs2_deg, gamma_derivative);
        gamma_deriv_plot.EdgeColor = 'none';
        colormap(gca, 'parula');
        cb = colorbar;
        title(cb, 'dV/dγ');
        hold on;
        
        % Add BRS/FRS boundary
        [~, h_data] = contour(xs1_deg, xs2_deg, current_data, [0 0], 'LineWidth', 2, 'Color', 'k');
        
        % Add zero-crossing contour for the derivative
        [~, h_zero] = contour(xs1_deg, xs2_deg, gamma_derivative, [0 0], 'LineWidth', 1.5, 'Color', 'r', 'LineStyle', '--');
        
        title('Derivative w.r.t. Yaw Rate (γ)', 'FontSize', 12);
        xlabel('Sideslip Angle (degrees)', 'FontSize', 10);
        ylabel('Yaw Rate (degrees/s)', 'FontSize', 10);
        legend([h_data, h_zero], [upper(computation_type) ' Boundary'], 'dV/dγ = 0', 'Location', 'best');
        grid on;
        
        % 3. Plot derivative magnitude (logarithmic scale)
        subplot(2, 2, 3);
        % Calculate gradient magnitude (Euclidean norm)
        gradient_magnitude = sqrt(gamma_derivative.^2 + beta_derivative.^2);
     
        % Apply logarithmic scaling (add small epsilon to avoid log(0))
        epsilon = 1e-10;
        log_magnitude = log10(gradient_magnitude + epsilon);
     
        % Plot with logarithmic scale
        magnitude_plot = pcolor(xs1_deg, xs2_deg, log_magnitude);
        magnitude_plot.EdgeColor = 'none';
        colormap(gca, 'hot');
        cb = colorbar;
        title(cb, 'log_{10}(||∇V||)');
        hold on;
     
        % Add BRS/FRS boundary
        [~, h_data] = contour(xs1_deg, xs2_deg, current_data, [0 0], 'LineWidth', 2, 'Color', 'k');
     
        title('Gradient Magnitude (Log Scale)', 'FontSize', 12);
        xlabel('Sideslip Angle (degrees)', 'FontSize', 10);
        ylabel('Yaw Rate (degrees/s)', 'FontSize', 10);
        grid on;
         
        % 4. Vector field visualization of the gradient
        subplot(2, 2, 4);
        % Create a coarser grid for the quiver plot
        [X, Y] = meshgrid(linspace(min(xs1_deg(:)), max(xs1_deg(:)), 20), ...
                          linspace(min(xs2_deg(:)), max(xs2_deg(:)), 20));
         
        % Interpolate gradient values to the coarser grid
        interp_gamma_deriv = interp2(xs1_deg, xs2_deg, gamma_derivative, X, Y);
        interp_beta_deriv = interp2(xs1_deg, xs2_deg, beta_derivative, X, Y);
         
        % Normalize for better visualization
        magnitudes = sqrt(interp_gamma_deriv.^2 + interp_beta_deriv.^2);
        max_mag = max(magnitudes(:));
        norm_factor = 1 / max_mag;
         
        norm_gamma_deriv = interp_gamma_deriv * norm_factor;
        norm_beta_deriv = interp_beta_deriv * norm_factor;
         
        % Create vector field
        h_quiver = quiver(X, Y, -norm_beta_deriv, -norm_gamma_deriv, 0.5, 'k');
        hold on;
         
        % Add BRS/FRS boundary
        [~, h_data] = contour(xs1_deg, xs2_deg, current_data, [0 0], 'LineWidth', 2, 'Color', 'k');
         
        % Add target set
        [~, h_target] = contour(xs1_deg, xs2_deg, data0, [0 0], 'LineWidth', 1.5, 'Color', 'g', 'LineStyle', '--');
         
        title('Gradient Vector Field ∇V', 'FontSize', 12);
        xlabel('Sideslip Angle (degrees)', 'FontSize', 10);
        ylabel('Yaw Rate (degrees/s)', 'FontSize', 10);
        legend([h_data, h_target], [upper(computation_type) ' Boundary'], 'Target Set', 'Location', 'best');
        grid on;
        
        % Add a main title for the entire figure
        if strcmp(control_type, 'mz')
            sgtitle(sprintf('Gradient Analysis for v = %d m/s, Mzmax = %d N·m', ...
                velocities(v_idx), control_limits(m_idx)), 'FontSize', 14, 'FontWeight', 'bold');
        else
            sgtitle(sprintf('Gradient Analysis for v = %d m/s, dvmax = %.1f deg/s', ...
                velocities(v_idx), control_limits(m_idx)*180/pi), 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        % Save figure if enabled
        if opts.saveFigs
            if strcmp(control_type, 'mz')
                fig_filename = sprintf('derivative_v%d_mz%d.%s', ...
                    velocities(v_idx), control_limits(m_idx), opts.figFormat);
            else
                fig_filename = sprintf('derivative_v%d_dv%.0f.%s', ...
                    velocities(v_idx), control_limits(m_idx)*180/pi, opts.figFormat);
            end
            saveas(gcf, fullfile(fig_folder, fig_filename));
            fprintf('Saved derivative plot to %s\n', fullfile(fig_folder, fig_filename));
        end
    end

%% Nested function to generate interactive tire force curve visualization
    function generate_tire_curves()
        disp('Generating interactive tire force curve visualization...');
        
        % This plot only applies to the 2D yaw moment control model
        if ~strcmp(control_type, 'mz')
            warning('Tire force visualization is only available for yaw moment control model.');
            return;
        end
        
        % Get indices to visualize
        v_idx = opts.velocityIdx;
        m_idx = opts.controlIdx;
        
        % Check if indices are valid
        if v_idx > length(velocities) || m_idx > length(control_limits)
            warning('Invalid indices for visualization. Using defaults.');
            v_idx = 1;
            m_idx = 1;
        end
        
        % Create a figure with specified layout
        fig = figure('Name', sprintf('Tire Force Analysis - v=%d, Mz=%d', velocities(v_idx), control_limits(m_idx)), ...
                     'Position', [100, 100, 1200, 700]);
        
        % Create a layout with left panel and right panel split vertically
        left_panel = subplot(1, 2, 1);
        
        % Create right panel and get its position
        right_panel = subplot(1, 2, 2); 
        p = get(right_panel, 'Position');
        delete(right_panel); % Delete it so we can create two stacked panels
        
        % Create two stacked panels on the right
        right_top = axes('Position', [p(1) p(2)+p(4)/2 p(3) p(4)/2-0.02]);
        right_bottom = axes('Position', [p(1) p(2) p(3) p(4)/2-0.02]);
        
        % Define slip angle range for tire curves
        alpha_range = linspace(-0.5, 0.5, 500); % ±30 degrees in radians
        alpha_deg = alpha_range * 180/pi;
        
        %% Setup vehicle parameters from the data
        % Get vehicle parameters from base_params
        base_params = result_data.base_params;
        
        m = base_params(1);      % Vehicle mass
        vx = velocities(v_idx);  % Current longitudinal velocity
        Lf = base_params(3);     % Distance from CG to front axle
        Lr = base_params(4);     % Distance from CG to rear axle
        Iz = base_params(5);     % Yaw moment of inertia
        mu = base_params(6);     % Friction coefficient
        Cf = base_params(9);     % Front tire cornering stiffness
        Cr = base_params(10);    % Rear tire cornering stiffness
        
        % Create a temporary NonlinearBicycle model to use its tire force calculation
        temp_params = base_params;
        temp_params(2) = vx; % Set the velocity parameter
        temp_model = NonlinearBicycle([0; 0], temp_params);
        
        %% Calculate tire forces for the slip angle range
        front_forces = zeros(size(alpha_range));
        rear_forces = zeros(size(alpha_range));
        
        for i = 1:length(alpha_range)
            % Calculate tire forces using a helper function
            front_forces(i) = calculateTireForce(alpha_range(i), temp_model.Cf, temp_model.Fzf, mu);
            rear_forces(i) = calculateTireForce(alpha_range(i), temp_model.Cr, temp_model.Fzr, mu);
        end
        
        %% Plot the BRS/FRS with current slice
        axes(left_panel);
        
        % Convert grid values from radians to degrees for plotting
        xs1_deg = g.xs{2} * 180/pi;  % Convert sideslip angle to degrees
        xs2_deg = g.xs{1} * 180/pi;  % Convert yaw rate to degrees
        
        % Plot the boundary
        current_brs = all_data{v_idx, m_idx};
        [~, h_brs] = contour(xs1_deg, xs2_deg, current_brs, [0 0], 'LineWidth', 2, 'Color', 'k');
        hold on;
        
        % Plot the target set
        [~, h_target] = contour(xs1_deg, xs2_deg, data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
        
        % Create a marker for the selected point (initially hidden)
        h_point = plot(0, 0, 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'Visible', 'off');
        
        % Add grid, labels, and legend
        grid on;
        xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
        ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
        title([upper(computation_type) ' Boundary - Click to Analyze'], 'FontSize', 14);
        legend([h_brs, h_target], [upper(computation_type) ' Boundary'], 'Target Set', 'Location', 'best');
        
        % Set axis limits
        x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
        y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
        xlim(x_limits);
        ylim(y_limits);
        
        %% Plot the front tire force curve
        axes(right_top);
        h_front_curve = plot(alpha_deg, front_forces / 1000, 'b-', 'LineWidth', 2);
        hold on;
        h_front_point = plot(0, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'Visible', 'off');
        grid on;
        
        % Calculate front tire saturation point
        front_sat_angle = atan(3*mu*temp_model.Fzf/Cf) * 180/pi;
        
        % Draw vertical lines at saturation angles
        front_sat_y_limits = [min(front_forces/1000), max(front_forces/1000)];
        plot([-front_sat_angle, -front_sat_angle], front_sat_y_limits, 'k--', 'LineWidth', 1);
        plot([front_sat_angle, front_sat_angle], front_sat_y_limits, 'k--', 'LineWidth', 1);
        
        xlabel('Slip Angle (degrees)', 'FontSize', 10);
        ylabel('Lateral Force (kN)', 'FontSize', 10);
        title('Front Tire Force vs. Slip Angle', 'FontSize', 12);
        
        %% Plot the rear tire force curve
        axes(right_bottom);
        h_rear_curve = plot(alpha_deg, rear_forces / 1000, 'r-', 'LineWidth', 2);
        hold on;
        h_rear_point = plot(0, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'Visible', 'off');
        grid on;
        
        % Calculate rear tire saturation point
        rear_sat_angle = atan(3*mu*temp_model.Fzr/Cr) * 180/pi;
        
        % Draw vertical lines at saturation angles
        rear_sat_y_limits = [min(rear_forces/1000), max(rear_forces/1000)];
        plot([-rear_sat_angle, -rear_sat_angle], rear_sat_y_limits, 'k--', 'LineWidth', 1);
        plot([rear_sat_angle, rear_sat_angle], rear_sat_y_limits, 'k--', 'LineWidth', 1);
        
        xlabel('Slip Angle (degrees)', 'FontSize', 10);
        ylabel('Lateral Force (kN)', 'FontSize', 10);
        title('Rear Tire Force vs. Slip Angle', 'FontSize', 12);
        
        %% Add a text box for numerical values
        text_panel = uicontrol('Style', 'text', ...
                              'String', 'Click on the BRS to analyze tire forces', ...
                              'Position', [900, 20, 250, 100], ...
                              'BackgroundColor', 'white', ...
                              'HorizontalAlignment', 'left', ...
                              'FontSize', 10);
        
        %% Setup interactivity
        % Store handles and parameters for callback function
        data_for_callback = struct();
        data_for_callback.h_point = h_point;
        data_for_callback.h_front_point = h_front_point;
        data_for_callback.h_rear_point = h_rear_point;
        data_for_callback.text_panel = text_panel;
        data_for_callback.vx = vx;
        data_for_callback.Lf = Lf;
        data_for_callback.Lr = Lr;
        data_for_callback.m = m;
        data_for_callback.Iz = Iz;
        data_for_callback.mu = mu;
        data_for_callback.Cf = Cf;
        data_for_callback.Cr = Cr;
        data_for_callback.Fzf = temp_model.Fzf;
        data_for_callback.Fzr = temp_model.Fzr;
        data_for_callback.front_sat_angle = front_sat_angle;
        data_for_callback.rear_sat_angle = rear_sat_angle;
        
        % Set callback for mouse clicks on the BRS plot
        set(fig, 'UserData', data_for_callback);
        set(left_panel, 'ButtonDownFcn', @(src, event) analyzeTirePoint(src, event, fig));
        
        % Add title to the figure
        sgtitle(sprintf('Tire Force Analysis for v = %d m/s, Mzmax = %d N·m', ...
               velocities(v_idx), control_limits(m_idx)), 'FontSize', 14, 'FontWeight', 'bold');
        
        % Save figure if enabled
        if opts.saveFigs
            fig_filename = sprintf('tire_curves_v%d_mz%d.%s', ...
                velocities(v_idx), control_limits(m_idx), opts.figFormat);
            saveas(fig, fullfile(fig_folder, fig_filename));
            fprintf('Saved tire curve plot to %s\n', fullfile(fig_folder, fig_filename));
        end
    end

%% Nested function for 3D slice plots (steered bicycle model)
    function generate_3d_slice_plots()
        disp('Generating 2D slice plots for 3D model...');
        
        % Get the 3rd dimension values (steering angle)
        delta_values = g.xs{3}(1, 1, :);
        
        % Determine slice indices based on requested steering angles
        delta_slices = opts.deltaSlices;
        slice_indices = zeros(size(delta_slices));
        
        for s = 1:length(delta_slices)
            [~, slice_indices(s)] = min(abs(delta_values - delta_slices(s)));
        end
        
        % Get indices to visualize
        v_idx = opts.velocityIdx;
        d_idx = opts.controlIdx;
        
        % Check if indices are valid
        if v_idx > length(velocities) || d_idx > length(control_limits)
            warning('Invalid indices for visualization. Using defaults.');
            v_idx = 1;
            d_idx = 1;
        end
        
        % Get data for this combination
        current_data = all_data{v_idx, d_idx};
        
        if ~isempty(control_data)
            current_control = control_data{v_idx, d_idx};
        else
            current_control = [];
        end
        
        % Create a figure for this combination
        figure('Name', sprintf('%s Slices - v=%d, dvmax=%.1f°/s', ...
            upper(computation_type), velocities(v_idx), control_limits(d_idx)*180/pi));
        
        % Position the figure
        set(gcf, 'Position', [100, 100, 300*length(slice_indices), 1000]);

        % Center index to find the target
        center_slice_idx = ceil(size(g.xs{3}, 3) / 2);
        
        % Create one subplot for each steering angle slice
        for s = 1:length(slice_indices)
            slice_idx = slice_indices(s);
            delta_val = delta_values(slice_idx);
            
            % Extract 2D slice
            data_slice = squeeze(current_data(:,:,slice_idx));
            target_slice = squeeze(data0(:,:,center_slice_idx));
            
            if ~isempty(current_control)
                control_slice = squeeze(current_control(:,:,slice_idx));
            else
                control_slice = [];
            end
            
            % Create subplot
            subplot(1, length(slice_indices), s);
            
            % Convert grid values from radians to degrees for plotting
            xs1_deg = g.xs{1}(:,:,1) * 180/pi;  % Convert yaw rate to degrees
            xs2_deg = g.xs{2}(:,:,1) * 180/pi;  % Convert sideslip angle to degrees
            
            if ~isempty(control_slice)
                % Plot control input with colored background
                % Put sideslip angle on x-axis, yaw rate on y-axis
                control_plot = pcolor(xs2_deg, xs1_deg, control_slice);
                control_plot.EdgeColor = 'none';
                colormap(gca, custom_cmap);
                
                % Add colorbar
                cb = colorbar;
                title(cb, 'Steering Rate (rad/s)');
                caxis([-control_limits(d_idx), control_limits(d_idx)]);
                
                hold on;
            end
            
            % Plot BRS/FRS boundary - sideslip on x, yaw rate on y
            [~, h_data] = contour(xs2_deg, xs1_deg, data_slice, [0 0], 'LineWidth', 2, 'Color', 'k');
            hold on;
            
            % Plot target set - sideslip on x, yaw rate on y
            [~, h_target] = contour(xs2_deg, xs1_deg, target_slice, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
            
            % Add labels, title, and legend
            xlabel('Sideslip Angle (deg)', 'FontSize', 12);
            ylabel('Yaw Rate (deg/s)', 'FontSize', 12);
            title(sprintf('Steering Angle = %.1f°', delta_val*180/pi), 'FontSize', 14);
            legend([h_data, h_target], [upper(computation_type) ' Boundary'], 'Target Set', 'Location', 'best');
            
            % Add grid
            grid on;
            
            % Calculate axis limits - sideslip for x, yaw rate for y
            x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
            y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
            
            % Set axis limits
            xlim(x_limits);
            ylim(y_limits);
        end
        
        % Add a main title for the entire figure
        sgtitle(sprintf('%s Slices for v = %d m/s, Max Steering Rate = %.1f°/s', ...
            upper(computation_type), velocities(v_idx), control_limits(d_idx)*180/pi), 'FontSize', 16, 'FontWeight', 'bold');
        
        % Save figure if enabled
        if opts.saveFigs
            fig_filename = sprintf('slices_v%d_dvmax%.0f.%s', ...
                velocities(v_idx), control_limits(d_idx)*180/pi, opts.figFormat);
            saveas(gcf, fullfile(fig_folder, fig_filename));
            fprintf('Saved slice plot to %s\n', fullfile(fig_folder, fig_filename));
        end
    end

%% Nested function for 3D detailed plots
    function generate_3d_detailed_plots()
        disp('Generating detailed analysis plots for 3D model...');
        
        % Get the 3rd dimension values (steering angle)
        delta_values = g.xs{3}(1, 1, :);
        
        % Use center slice for detailed plots
        center_slice_idx = ceil(size(g.xs{3}, 3) / 2);
        delta_val = delta_values(center_slice_idx);
        
        % Get indices to visualize
        v_idx = opts.velocityIdx;
        d_idx = opts.controlIdx;
        
        % Check if indices are valid
        if v_idx > length(velocities) || d_idx > length(control_limits)
            warning('Invalid indices for visualization. Using defaults.');
            v_idx = 1;
            d_idx = 1;
        end
        
        % Get data for this combination
        current_data = all_data{v_idx, d_idx};
        
        if ~isempty(control_data)
            current_control = control_data{v_idx, d_idx};
        else
            current_control = [];
        end
        
        % Extract center slice
        data_slice = squeeze(current_data(:,:,center_slice_idx));
        target_slice = squeeze(data0(:,:,center_slice_idx));
        
        if ~isempty(current_control)
            control_slice = squeeze(current_control(:,:,center_slice_idx));
        else
            control_slice = [];
        end
        
        % Create figure
        figure('Name', sprintf('Detailed - v=%d, dvmax=%.1f°/s, delta=%.1f°', ...
            velocities(v_idx), control_limits(d_idx)*180/pi, delta_val*180/pi));
        
        % Position the figure
        set(gcf, 'Position', [100, 100, 900, 700]);
        
        % Convert grid values from radians to degrees for plotting
        xs1_deg = g.xs{1}(:,:,1) * 180/pi;  % Convert yaw rate to degrees
        xs2_deg = g.xs{2}(:,:,1) * 180/pi;  % Convert sideslip angle to degrees
        
        % Calculate axis limits - sideslip for x, yaw rate for y
        x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
        y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
        
        % 1. Control Input Visualization
        subplot(2, 2, 1);
        
        if ~isempty(control_slice)
            % Create a pcolor plot with the custom colormap - sideslip on x, yaw rate on y
            control_plot = pcolor(xs2_deg, xs1_deg, control_slice);
            control_plot.EdgeColor = 'none';
            colormap(gca, custom_cmap);
            caxis([-control_limits(d_idx), control_limits(d_idx)]);
            
            % Add contour of BRS/FRS boundary
            hold on;
            [~, h_data] = contour(xs2_deg, xs1_deg, data_slice, [0 0], 'LineWidth', 2, 'Color', 'k');
            
            % Add colorbar and labels
            cb = colorbar;
            title(cb, 'Steering Rate (rad/s)');
            
            title('Optimal Control Input', 'FontSize', 12);
        else
            % If control data is not available, show blank panel with message
            text(0.5, 0.5, 'Control data not available', 'HorizontalAlignment', 'center', 'FontSize', 12);
            axis off;
        end
        
        xlabel('Sideslip Angle (deg)', 'FontSize', 10);
        ylabel('Yaw Rate (deg/s)', 'FontSize', 10);
        xlim(x_limits);
        ylim(y_limits);
        grid on;
        
        % 2. BRS/FRS Boundary with Control Switching Surface
        subplot(2, 2, 2);
        
        % Plot BRS/FRS boundary - sideslip on x, yaw rate on y
        [~, h_data] = contour(xs2_deg, xs1_deg, data_slice, [0 0], 'LineWidth', 2, 'Color', 'k');
        hold on;
        
        % Plot target set - sideslip on x, yaw rate on y
        [~, h_target] = contour(xs2_deg, xs1_deg, target_slice, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
        
        % Find and plot the control switching surface if we have control data
        if ~isempty(control_slice)
            % For 3D BRS, need to extract 2D derivatives
            derivs = extract2DCostates(g, data_slice);
            switching_condition = derivs{2};  % The derivative w.r.t. delta
            
            % Plot switching surface
            [~, h_switch] = contour(xs2_deg, xs1_deg, switching_condition, [0 0], 'LineWidth', 1.5, 'Color', 'r', 'LineStyle', '-.'); 
            
            title([upper(computation_type) ' Boundary and Control Switching Surface'], 'FontSize', 12);
            legend([h_data, h_target, h_switch], [upper(computation_type) ' Boundary'], 'Target Set', 'Control Switching', 'Location', 'best', 'FontSize', 8);
        else
            title([upper(computation_type) ' Boundary'], 'FontSize', 12);
            legend([h_data, h_target], [upper(computation_type) ' Boundary'], 'Target Set', 'Location', 'best', 'FontSize', 8);
        end
        
        xlabel('Sideslip Angle (deg)', 'FontSize', 10);
        ylabel('Yaw Rate (deg/s)', 'FontSize', 10);
        xlim(x_limits);
        ylim(y_limits);
        grid on;
        
        % 3. Value Function Surface
        subplot(2, 2, 3:4);
        
        % Plot the value function as a surface - sideslip on x, yaw rate on y
        surf(xs2_deg, xs1_deg, data_slice, 'EdgeColor', 'none');
        colormap(gca, parula);
        
        % Adjust view angle
        view([-30, 30]);
        
        % Add the zero level set (BRS/FRS boundary)
        hold on;
        [~, h_data] = contour3(xs2_deg, xs1_deg, data_slice, [0 0], 'LineWidth', 2, 'Color', 'r');
        
        % Add colorbar and labels
        cb = colorbar;
        title(cb, 'Value Function');
        
        title('Value Function Surface', 'FontSize', 12);
        xlabel('Sideslip Angle (deg)', 'FontSize', 10);
        ylabel('Yaw Rate (deg/s)', 'FontSize', 10);
        zlabel('Value', 'FontSize', 10);
        grid on;
        
        % Add a main title for the entire figure
        sgtitle(sprintf('Detailed Analysis (v = %d m/s, Max Steering Rate = %.1f°/s, δ = %.1f°)', ...
                velocities(v_idx), control_limits(d_idx)*180/pi, delta_val*180/pi), 'FontSize', 14, 'FontWeight', 'bold');
        
        % Save figure if enabled
        if opts.saveFigs
            fig_filename = sprintf('detailed_v%d_dvmax%.0f_delta%.0f.%s', ...
                velocities(v_idx), control_limits(d_idx)*180/pi, delta_val*180/pi, opts.figFormat);
            saveas(gcf, fullfile(fig_folder, fig_filename));
            fprintf('Saved detailed plot to %s\n', fullfile(fig_folder, fig_filename));
        end
    end

%% Nested function for 3D comparison plots
    function generate_3d_comparison_plots()
        disp('Generating comparison plots for 3D model...');
        
        % Get the 3rd dimension values (steering angle)
        delta_values = g.xs{3}(1, 1, :);
        
        % Use center slice for comparisons
        center_slice_idx = ceil(size(g.xs{3}, 3) / 2);
        delta_val = delta_values(center_slice_idx);
        
        % Comparison of different steering rates (if applicable)
        if length(control_limits) > 1
            % Compare for different steering rates with the same velocity
            v_idx = opts.velocityIdx;
            
            if v_idx > length(velocities)
                warning('Invalid velocity index. Using first velocity.');
                v_idx = 1;
            end
            
            figure('Name', sprintf('dvmax Comparison - v=%d, delta=%.1f°', ...
                velocities(v_idx), delta_val*180/pi));
            clf;
            
            % Create a larger figure
            set(gcf, 'Position', [100, 100, 800, 600]);
            
            % Convert grid values from radians to degrees for plotting
            xs1_deg = g.xs{1}(:,:,1) * 180/pi;
            xs2_deg = g.xs{2}(:,:,1) * 180/pi;
            
            % Calculate axis limits - sideslip for x, yaw rate for y
            x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
            y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
            
            % Extract target set slice
            target_slice = squeeze(data0(:,:,center_slice_idx));
            
            % Plot target set - sideslip on x, yaw rate on y
            [~, h_target] = contour(xs2_deg, xs1_deg, target_slice, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
            hold on;
            
            % Create color map for different steering rate values
            dv_colors = jet(length(control_limits));
            
            % Plot BRS/FRS boundary for each steering rate
            h_data = zeros(length(control_limits), 1);
            for d_idx = 1:length(control_limits)
                % Extract data slice
                data_slice = squeeze(all_data{v_idx, d_idx}(:,:,center_slice_idx));
                
                % Sideslip on x, yaw rate on y
                [~, h_data(d_idx)] = contour(xs2_deg, xs1_deg, data_slice, [0 0], ...
                    'LineWidth', 2, 'Color', dv_colors(d_idx,:));
            end
            
            % Create legend
            legend_entries = cell(length(control_limits) + 1, 1);
            for d_idx = 1:length(control_limits)
                legend_entries{d_idx} = sprintf('dv_max = %.0f°/s', control_limits(d_idx)*180/pi);
            end
            legend_entries{end} = 'Target Set';
            
            % Add labels, title and legend
            grid on;
            xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
            ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
            title(sprintf('%s Comparison for Different Steering Rate Limits (v = %d m/s, δ = %.1f°)', ...
                  upper(computation_type), velocities(v_idx), delta_val*180/pi), 'FontSize', 14, 'FontWeight', 'bold');
            legend([h_data; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
            
            % Add a better grid
            grid minor;
            
            % Set axis limits
            xlim(x_limits);
            ylim(y_limits);
            
            % Save figure if enabled
            if opts.saveFigs
                fig_filename = sprintf('dvmax_comparison_v%d_delta%.0f.%s', ...
                    velocities(v_idx), delta_val*180/pi, opts.figFormat);
                saveas(gcf, fullfile(fig_folder, fig_filename));
                fprintf('Saved steering rate comparison plot to %s\n', fullfile(fig_folder, fig_filename));
            end
        end
        
        % Comparison of different velocities (if applicable)
        if length(velocities) > 1
            % Compare for different velocities with the same steering rate
            d_idx = opts.controlIdx;
            
            if d_idx > length(control_limits)
                warning('Invalid control index. Using first control limit.');
                d_idx = 1;
            end
            
            figure('Name', sprintf('Velocity Comparison - dvmax=%.1f°/s, delta=%.1f°', ...
                control_limits(d_idx)*180/pi, delta_val*180/pi));
            clf;
            
            % Create a larger figure
            set(gcf, 'Position', [100, 100, 800, 600]);
            
            % Convert grid values from radians to degrees for plotting
            xs1_deg = g.xs{1}(:,:,1) * 180/pi;
            xs2_deg = g.xs{2}(:,:,1) * 180/pi;
            
            % Calculate axis limits
            x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
            y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
            
            % Extract target set slice
            target_slice = squeeze(data0(:,:,center_slice_idx));
            
            % Plot target set
            [~, h_target] = contour(xs2_deg, xs1_deg, target_slice, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
            hold on;
            
            % Create color map for different velocities
            vel_colors = jet(length(velocities));
            
            % Plot boundary for each velocity
            h_data = zeros(length(velocities), 1);
            for v_idx = 1:length(velocities)
                % Extract BRS/FRS slice
                data_slice = squeeze(all_data{v_idx, d_idx}(:,:,center_slice_idx));
                
                [~, h_data(v_idx)] = contour(xs2_deg, xs1_deg, data_slice, [0 0], ...
                    'LineWidth', 2, 'Color', vel_colors(v_idx,:));
            end
            
            % Create legend
            legend_entries = cell(length(velocities) + 1, 1);
            for v_idx = 1:length(velocities)
                legend_entries{v_idx} = sprintf('v = %d m/s', velocities(v_idx));
            end
            legend_entries{end} = 'Target Set';
            
            % Add labels, title and legend
            grid on;
            xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
            ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
            title(sprintf('%s Comparison for Different Velocities (Max Steering Rate = %.1f°/s, δ = %.1f°)', ...
                  upper(computation_type), control_limits(d_idx)*180/pi, delta_val*180/pi), 'FontSize', 14, 'FontWeight', 'bold');
            legend([h_data; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
            
            % Add a better grid
            grid minor;
            
            % Set axis limits
            xlim(x_limits);
            ylim(y_limits);
            
            % Save figure if enabled
            if opts.saveFigs
                fig_filename = sprintf('velocity_comparison_dvmax%.0f_delta%.0f.%s', ...
                    control_limits(d_idx)*180/pi, delta_val*180/pi, opts.figFormat);
                saveas(gcf, fullfile(fig_folder, fig_filename));
                fprintf('Saved velocity comparison plot to %s\n', fullfile(fig_folder, fig_filename));
            end
        end
        
        % Generate delta slices comparison for the first velocity and steering rate
        v_idx = opts.velocityIdx;
        d_idx = opts.controlIdx;
        
        if v_idx > length(velocities) || d_idx > length(control_limits)
            warning('Invalid indices for delta slice comparison. Using defaults.');
            v_idx = min(v_idx, length(velocities));
            d_idx = min(d_idx, length(control_limits));
        end
        
        % Get the data
        current_data = all_data{v_idx, d_idx};
        
        % Select a few delta slices to compare
        num_slices = 5;
        delta_indices = round(linspace(1, size(g.xs{3}, 3), num_slices));
        
        figure('Name', sprintf('Delta Slice Comparison - v=%d, dvmax=%.1f°/s', ...
            velocities(v_idx), control_limits(d_idx)*180/pi));
        clf;
        
        % Create a larger figure
        set(gcf, 'Position', [100, 100, 800, 600]);
        
        % Convert grid values from radians to degrees for plotting
        xs1_deg = g.xs{1}(:,:,1) * 180/pi;
        xs2_deg = g.xs{2}(:,:,1) * 180/pi;
        
        % Calculate axis limits
        x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
        y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
        
        % Plot target set (use middle slice as reference)
        center_slice_idx = ceil(size(g.xs{3}, 3) / 2);
        target_slice = squeeze(data0(:,:,center_slice_idx));
        
        % Sideslip on x, yaw rate on y
        [~, h_target] = contour(xs2_deg, xs1_deg, target_slice, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
        hold on;
        
        % Create color map for different delta slices
        delta_colors = jet(length(delta_indices));

        % Create legend
        legend_entries = cell(length(delta_indices) + 1, 1);
        
        % Plot BRS boundary for each delta slice
        h_data = zeros(length(delta_indices), 1);
        for s = 1:length(delta_indices)
            slice_idx = delta_indices(s);
            delta_val = delta_values(slice_idx);
            
            % Extract FRS slice
            data_slice = squeeze(current_data(:,:,slice_idx));
            
            % Sideslip on x, yaw rate on y
            [~, h_data(s)] = contour(xs2_deg, xs1_deg, data_slice, [0 0], ...
                'LineWidth', 2, 'Color', delta_colors(s,:));

            % Create legend entry
            legend_entries{s} = sprintf('δ = %.1f°', delta_val*180/pi);
        end
        
        legend_entries{end} = 'Target Set';
        
        % Add labels, title and legend
        grid on;
        xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
        ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
        title(sprintf('%s Comparison for Different Steering Angles (v = %d m/s, Max Steering Rate = %.1f°/s)', ...
              upper(computation_type), velocities(v_idx), control_limits(d_idx)*180/pi), 'FontSize', 14, 'FontWeight', 'bold');
        legend([h_data; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
        
        % Add a better grid
        grid minor;
        
        % Set axis limits
        xlim(x_limits);
        ylim(y_limits);
        
        % Save figure if enabled
        if opts.saveFigs
            fig_filename = sprintf('delta_comparison_v%d_dvmax%.0f.%s', ...
                velocities(v_idx), control_limits(d_idx)*180/pi, opts.figFormat);
            saveas(gcf, fullfile(fig_folder, fig_filename));
            fprintf('Saved delta comparison plot to %s\n', fullfile(fig_folder, fig_filename));
        end
    end

%% Helper function to calculate tire forces (similar to the one in NonlinearBicycle)
function Fy = calculateTireForce(alpha, C, Fz, mu)
    % Threshold for linear region
    threshold = atan(3*mu*Fz/C);
    
    if abs(alpha) < threshold
        % Linear region with smoothing toward saturation
        Fy = -C * alpha + C^2/(3*mu*Fz) * abs(alpha) * alpha - ...
             (1/3) * C^3/(3*mu*Fz)^2 * alpha^3;
    else
        % Saturation region
        Fy = -mu*Fz * sign(alpha);
    end
end

%% Helper function for tire force analysis callback
function analyzeTirePoint(src, event, fig_handle)
    % Get the clicked point in axis coordinates
    point = get(src, 'CurrentPoint');
    x_click = point(1, 1); % Sideslip angle in degrees
    y_click = point(1, 2); % Yaw rate in degrees
    
    % Convert to radians for calculations
    beta = x_click * pi/180;
    gamma = y_click * pi/180;
    
    % Get stored data
    data = get(fig_handle, 'UserData');
    
    % Update point marker on BRS/FRS plot
    set(data.h_point, 'XData', x_click, 'YData', y_click, 'Visible', 'on');
    
    % Calculate slip angles
    % Front slip angle: alpha_f = beta + (Lf*gamma/vx)
    % Rear slip angle: alpha_r = beta - (Lr*gamma/vx)
    alpha_f = beta + (data.Lf*gamma/data.vx);
    alpha_r = beta - (data.Lr*gamma/data.vx);
    
    % Convert to degrees for display
    alpha_f_deg = alpha_f * 180/pi;
    alpha_r_deg = alpha_r * 180/pi;
    
    % Calculate tire forces
    Fyf = calculateTireForce(alpha_f, data.Cf, data.Fzf, data.mu);
    Fyr = calculateTireForce(alpha_r, data.Cr, data.Fzr, data.mu);
    
    % Calculate total lateral force and moment
    F_lat = Fyf + Fyr;
    M_z = data.Lf*Fyf - data.Lr*Fyr;
    
    % Calculate lateral acceleration
    ay = F_lat / data.m;
    
    % Update points on the tire force curves
    set(data.h_front_point, 'XData', alpha_f_deg, 'YData', Fyf/1000, 'Visible', 'on');
    set(data.h_rear_point, 'XData', alpha_r_deg, 'YData', Fyr/1000, 'Visible', 'on');
    
    % Calculate utilization percentages
    front_utilization = abs(Fyf)/(data.mu*data.Fzf)*100;
    rear_utilization = abs(Fyr)/(data.mu*data.Fzr)*100;
    
    % Update text box with numerical values
    info_text = sprintf(['Selected Point:\n', ...
                       'Sideslip Angle (β): %.2f°\n', ...
                       '  Yaw Rate (γ): %.2f°/s\n\n', ...
                       'Tire Slip Angles:\n', ...
                       '  Front (αf): %.2f° %s\n', ...
                       '  Rear (αr): %.2f° %s\n\n', ...
                       'Tire Forces:\n', ...
                       '  Front: %.2f kN\n', ...
                       '  Rear: %.2f kN\n', ...
                       '  Total Lat: %.2f kN\n', ...
                       '  Yaw Moment: %.2f kN·m\n\n', ...
                       'Force Utilization:\n', ...
                       '  Front: %.1f%%\n', ...
                       '  Rear: %.1f%%\n\n', ...
                       'Lat. Accel: %.2f m/s²'], ...
                       x_click, y_click, ...
                       alpha_f_deg, saturationStatus(abs(alpha_f_deg), data.front_sat_angle), ...
                       alpha_r_deg, saturationStatus(abs(alpha_r_deg), data.rear_sat_angle), ...
                       Fyf/1000, Fyr/1000, ...
                       F_lat/1000, M_z/1000, ...
                       front_utilization, ...
                       rear_utilization, ...
                       ay);
    
    set(data.text_panel, 'String', info_text);
end

% Helper function to show saturation status
function status = saturationStatus(angle, sat_angle)
    if angle >= sat_angle * 0.95
        status = '(saturated)';
    elseif angle >= sat_angle * 0.7
        status = '(near sat)';
    else
        status = '';
    end
end


%% Helper function to extract costates (gradients) from the value function
function derivs = extractCostates(g, data)
    % Initialize derivatives
    derivs = cell(g.dim, 1);
    
    % Use upwinding for proper gradient calculation
    for i = 1:g.dim
        % Initialize arrays for left and right derivatives
        deriv_left = zeros(size(data));
        deriv_right = zeros(size(data));
        
        % Calculate left and right derivatives with appropriate boundary handling
        if i == 1
            % For the first dimension (beta)
            deriv_left(2:end, :) = (data(2:end, :) - data(1:end-1, :)) / g.dx(i);
            deriv_right(1:end-1, :) = (data(2:end, :) - data(1:end-1, :)) / g.dx(i);
            
            % Handle boundaries with one-sided differences
            deriv_left(1, :) = (data(2, :) - data(1, :)) / g.dx(i);
            deriv_right(end, :) = (data(end, :) - data(end-1, :)) / g.dx(i);
        else
            % For the second dimension (gamma)
            deriv_left(:, 2:end) = (data(:, 2:end) - data(:, 1:end-1)) / g.dx(i);
            deriv_right(:, 1:end-1) = (data(:, 2:end) - data(:, 1:end-1)) / g.dx(i);
            
            % Handle boundaries with one-sided differences
            deriv_left(:, 1) = (data(:, 2) - data(:, 1)) / g.dx(i);
            deriv_right(:, end) = (data(:, end) - data(:, end-1)) / g.dx(i);
        end
        
        % Choose derivative based on the sign of the Hamiltonian
        % For target reaching, take the derivative that would lead to a larger Hamiltonian value
        if i == 1  % beta dimension
            % The sign would depend on the system dynamics - for simplicity,
            % we'll use central differencing here
            derivs{i} = (deriv_left + deriv_right) / 2;
        else  % gamma dimension - critical for control determination
            derivs{i} = deriv_left;
            derivs{i}(deriv_right < deriv_left) = deriv_right(deriv_right < deriv_left);
        end
    end
end

% The implementation of other functions (generate_detailed_plots, generate_comparison_plots, 
% generate_3d_velocity_stack) would remain unchanged from the original code.

end