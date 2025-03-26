function visualize_brs_results_steered(result_folder, varargin)
% VISUALIZE_BRS_RESULTS_STEERED Creates visualizations from saved BRS computation results
% for the 3-state NonlinearBicycleSteered model
%
% Inputs:
%   result_folder - Path to the folder containing BRS computation results
%   varargin      - Optional parameter-value pairs:
%                   'plotType'   - Types of plots to generate (cell array)
%                                 Options: 'slices', 'comparison', 'detailed'
%                                 (default: {'slices', 'comparison'})
%                   'saveFigs'   - Whether to save figures (default: true)
%                   'figFormat'  - Format to save figures in (default: 'png')
%                   'deltaSlices'- Specific steering angle slices to visualize 
%                                 (default: [-0.1, 0, 0.1] rad)
%
% Example:
%   visualize_brs_results_steered('steered_brs_results_20250325_123456_vx30-50_dvmax5-10');
%   visualize_brs_results_steered('results_folder', 'plotType', {'slices'}, 'deltaSlices', [-0.05, 0, 0.05]);

%% Parse inputs
p = inputParser;
p.addRequired('result_folder', @ischar);
p.addParameter('plotType', {'slices', 'comparison'}, @iscell);
p.addParameter('saveFigs', true, @islogical);
p.addParameter('figFormat', 'png', @ischar);
p.addParameter('deltaSlices', [-0.1, 0, 0.1], @isnumeric); % Steering angle slices in radians

p.parse(result_folder, varargin{:});
opts = p.Results;

%% Check if the folder exists
if ~exist(result_folder, 'dir')
    error('Result folder does not exist: %s', result_folder);
end

%% Load combined results
combined_file = fullfile(result_folder, 'brs_combined_results.mat');
if ~exist(combined_file, 'file')
    error('Combined results file not found: %s', combined_file);
end

disp(['Loading results from: ', combined_file]);
load(combined_file, 'g', 'data0', 'all_data', 'all_data_full', 'control_data', ...
     'velocities', 'dvmax_values', 'tau', 'base_params');

%% Load simulation parameters
sim_params_file = fullfile(result_folder, 'sim_params.mat');
if exist(sim_params_file, 'file')
    load(sim_params_file, 'sim_params');
    disp('Loaded simulation parameters.');
else
    warning('Simulation parameters file not found. Using defaults from combined results.');
    sim_params = struct();
    sim_params.velocities = velocities;
    sim_params.dvmax_values = dvmax_values;
    sim_params.tau = tau;
    sim_params.base_params = base_params;
end

%% Setup visualization parameters
% Create a fixed simple red/blue colormap for clear distinction between positive and negative control
custom_cmap = [
    0.7 0 0;    % Dark red for strong negative control
    1 0.5 0.5;  % Light red for weak negative control
    0.5 0.5 1;  % Light blue for weak positive control
    0 0 0.7     % Dark blue for strong positive control
];

%% Create output folder for figures if saving is enabled
if opts.saveFigs
    fig_folder = fullfile(result_folder, 'figures');
    if ~exist(fig_folder, 'dir')
        mkdir(fig_folder);
        disp(['Created figures directory: ', fig_folder]);
    end
end

%% Generate requested visualizations
plot_types = opts.plotType;

% Process each plot type
for i = 1:length(plot_types)
    current_plot_type = plot_types{i};
    
    switch lower(current_plot_type)
        case 'slices'
            generate_slice_plots();
            
        case 'detailed'
            generate_detailed_plots();
            
        case 'comparison'
            generate_comparison_plots();
            
        otherwise
            warning('Unknown plot type: %s', current_plot_type);
    end
end

disp('Visualization complete!');

%% Nested function to generate 2D slices of 3D BRS at different steering angles
    function generate_slice_plots()
        disp('Generating 2D slice plots at different steering angles...');
        
        % Get the 3rd dimension values (steering angle)
        delta_values = g.xs{3}(1, 1, :);
        
        % Determine slice indices based on requested steering angles
        delta_slices = opts.deltaSlices;
        slice_indices = zeros(size(delta_slices));
        
        for s = 1:length(delta_slices)
            [~, slice_indices(s)] = min(abs(delta_values - delta_slices(s)));
        end
        
        % Generate plots for each velocity and steering rate combination
        for v_idx = 1:length(velocities)
            for d_idx = 1:length(dvmax_values)
                % Get data for this combination
                current_brs = all_data{v_idx, d_idx};
                current_control = control_data{v_idx, d_idx};
                
                % Create a figure for this combination
                figure('Name', sprintf('BRS Slices - v=%d, dvmax=%.1f°/s', ...
                    velocities(v_idx), dvmax_values(d_idx)*180/pi));
                
                % Position the figure
                set(gcf, 'Position', [100, 100, 1000, 300*length(slice_indices)]);
                
                % Create one subplot for each steering angle slice
                for s = 1:length(slice_indices)
                    slice_idx = slice_indices(s);
                    delta_val = delta_values(slice_idx);
                    
                    % Extract 2D slice
                    brs_slice = squeeze(current_brs(:,:,slice_idx));
                    target_slice = squeeze(data0(:,:,slice_idx));
                    control_slice = squeeze(current_control(:,:,slice_idx));
                    
                    % Create subplot
                    subplot(length(slice_indices), 1, s);
                    
                    % Convert grid values from radians to degrees for plotting
                    xs1_deg = g.xs{1}(:,:,1) * 180/pi;  % Convert yaw rate to degrees
                    xs2_deg = g.xs{2}(:,:,1) * 180/pi;  % Convert sideslip angle to degrees
                    
                    % Plot control input with colored background
                    control_plot = pcolor(xs1_deg, xs2_deg, control_slice);
                    control_plot.EdgeColor = 'none';
                    colormap(gca, custom_cmap);
                    
                    % Add colorbar
                    cb = colorbar;
                    title(cb, 'Steering Rate (rad/s)');
                    caxis([-dvmax_values(d_idx), dvmax_values(d_idx)]);
                    
                    hold on;
                    
                    % Plot BRS boundary
                    [~, h_brs] = contour(xs1_deg, xs2_deg, brs_slice, [0 0], 'LineWidth', 2, 'Color', 'k');
                    
                    % Plot target set
                    [~, h_target] = contour(xs1_deg, xs2_deg, target_slice, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                    
                    % Add labels, title, and legend
                    xlabel('Yaw Rate (deg/s)', 'FontSize', 12);
                    ylabel('Sideslip Angle (deg)', 'FontSize', 12);
                    title(sprintf('Steering Angle = %.1f°', delta_val*180/pi), 'FontSize', 14);
                    legend([h_brs, h_target], 'BRS Boundary', 'Target Set', 'Location', 'best');
                    
                    % Add grid
                    grid on;
                    
                    % Calculate axis limits
                    x_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
                    y_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
                    
                    % Set axis limits
                    xlim(x_limits);
                    ylim(y_limits);
                end
                
                % Add a main title for the entire figure
                sgtitle(sprintf('BRS Slices for v = %d m/s, Max Steering Rate = %.1f°/s', ...
                    velocities(v_idx), dvmax_values(d_idx)*180/pi), 'FontSize', 16, 'FontWeight', 'bold');
                
                % Save figure if enabled
                if opts.saveFigs
                    fig_filename = sprintf('slices_v%d_dvmax%.0f.%s', ...
                        velocities(v_idx), dvmax_values(d_idx)*180/pi, opts.figFormat);
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                    fprintf('Saved slice plot to %s\n', fullfile(fig_folder, fig_filename));
                end
            end
        end
    end

%% Nested function to generate detailed plots with control strategies
    function generate_detailed_plots()
        disp('Generating detailed control analysis plots...');
        
        % Get the 3rd dimension values (steering angle)
        delta_values = g.xs{3}(1, 1, :);
        
        % Use center slice for detailed plots
        center_slice_idx = ceil(size(g.xs{3}, 1) / 2);
        delta_val = delta_values(center_slice_idx);
        
        for v_idx = 1:length(velocities)
            for d_idx = 1:length(dvmax_values)
                % Get data for this combination
                current_brs = all_data{v_idx, d_idx};
                current_control = control_data{v_idx, d_idx};
                
                % Extract center slice
                brs_slice = squeeze(current_brs(:,:,center_slice_idx));
                target_slice = squeeze(data0(:,:,center_slice_idx));
                control_slice = squeeze(current_control(:,:,center_slice_idx));
                
                % Create figure
                figure('Name', sprintf('Detailed - v=%d, dvmax=%.1f°/s, delta=%.1f°', ...
                    velocities(v_idx), dvmax_values(d_idx)*180/pi, delta_val*180/pi));
                
                % Position the figure
                set(gcf, 'Position', [100, 100, 900, 700]);
                
                % Convert grid values from radians to degrees for plotting
                xs1_deg = g.xs{1}(:,:,1) * 180/pi;  % Convert yaw rate to degrees
                xs2_deg = g.xs{2}(:,:,1) * 180/pi;  % Convert sideslip angle to degrees
                
                % Calculate axis limits
                x_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
                y_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
                
                % 1. Control Input Visualization
                subplot(2, 2, 1);
                
                % Create a pcolor plot with the custom colormap
                control_plot = pcolor(xs1_deg, xs2_deg, control_slice);
                control_plot.EdgeColor = 'none';
                colormap(gca, custom_cmap);
                caxis([-dvmax_values(d_idx), dvmax_values(d_idx)]);
                
                % Add contour of BRS boundary
                hold on;
                [~, h_brs] = contour(xs1_deg, xs2_deg, brs_slice, [0 0], 'LineWidth', 2, 'Color', 'k');
                
                % Add colorbar and labels
                cb = colorbar;
                title(cb, 'Steering Rate (rad/s)');
                
                title('Optimal Control Input', 'FontSize', 12);
                ylabel('Sideslip Angle (deg)', 'FontSize', 10);
                xlabel('Yaw Rate (deg/s)', 'FontSize', 10);
                xlim(x_limits);
                ylim(y_limits);
                grid on;
                
                % 2. BRS Boundary with Control Switching Surface
                subplot(2, 2, 2);
                
                % Plot BRS boundary
                [~, h_brs] = contour(xs1_deg, xs2_deg, brs_slice, [0 0], 'LineWidth', 2, 'Color', 'k');
                hold on;
                
                % Plot target set
                [~, h_target] = contour(xs1_deg, xs2_deg, target_slice, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                
                % Find and plot the control switching surface
                % For 3D BRS, need to extract 2D derivatives
                derivs = extract2DCostates(g, brs_slice);
                switching_condition = derivs{2};  % The derivative w.r.t. delta
                [~, h_switch] = contour(xs1_deg, xs2_deg, switching_condition, [0 0], 'LineWidth', 1.5, 'Color', 'r', 'LineStyle', '-.'); 
                
                title('BRS Boundary and Control Switching Surface', 'FontSize', 12);
                ylabel('Sideslip Angle (deg)', 'FontSize', 10);
                xlabel('Yaw Rate (deg/s)', 'FontSize', 10);
                legend([h_brs, h_target, h_switch], 'BRS Boundary', 'Target Set', 'Control Switching', 'Location', 'best', 'FontSize', 8);
                xlim(x_limits);
                ylim(y_limits);
                grid on;
                
                % 3. Value Function Surface
                subplot(2, 2, 3:4);
                
                % Plot the value function as a surface
                surf(xs1_deg, xs2_deg, brs_slice, 'EdgeColor', 'none');
                colormap(gca, parula);
                
                % Adjust view angle
                view([-30, 30]);
                
                % Add the zero level set (BRS boundary)
                hold on;
                [~, h_brs] = contour3(xs1_deg, xs2_deg, brs_slice, [0 0], 'LineWidth', 2, 'Color', 'r');
                
                % Add colorbar and labels
                cb = colorbar;
                title(cb, 'Value Function');
                
                title('Value Function Surface', 'FontSize', 12);
                xlabel('Yaw Rate (deg/s)', 'FontSize', 10);
                ylabel('Sideslip Angle (deg)', 'FontSize', 10);
                zlabel('Value', 'FontSize', 10);
                grid on;
                
                % Add a main title for the entire figure
                sgtitle(sprintf('Detailed Control Analysis (v = %d m/s, Max Steering Rate = %.1f°/s, δ = %.1f°)', ...
                        velocities(v_idx), dvmax_values(d_idx)*180/pi, delta_val*180/pi), 'FontSize', 14, 'FontWeight', 'bold');
                
                % Save figure if enabled
                if opts.saveFigs
                    fig_filename = sprintf('detailed_v%d_dvmax%.0f_delta%.0f.%s', ...
                        velocities(v_idx), dvmax_values(d_idx)*180/pi, delta_val*180/pi, opts.figFormat);
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                    fprintf('Saved detailed plot to %s\n', fullfile(fig_folder, fig_filename));
                end
            end
        end
    end

%% Nested function to generate comparison plots across different parameters
    function generate_comparison_plots()
        disp('Generating comparison plots...');
        
        % Get the 3rd dimension values (steering angle)
        delta_values = g.xs{3}(1, 1, :);
        
        % Use center slice for comparisons
        center_slice_idx = ceil(size(g.xs{3}, 1) / 2);
        delta_val = delta_values(center_slice_idx);
        
        % Comparison of different steering rates (if applicable)
        if length(dvmax_values) > 1
            % Compare BRS for different steering rates with the same velocity
            for v_idx = 1:length(velocities)
                figure('Name', sprintf('dvmax Comparison - v=%d, delta=%.1f°', ...
                    velocities(v_idx), delta_val*180/pi));
                clf;
                
                % Create a larger figure
                set(gcf, 'Position', [100, 100, 800, 600]);
                
                % Convert grid values from radians to degrees for plotting
                xs1_deg = g.xs{1}(:,:,1) * 180/pi;
                xs2_deg = g.xs{2}(:,:,1) * 180/pi;
                
                % Calculate axis limits
                x_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
                y_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
                
                % Extract target set slice
                target_slice = squeeze(data0(:,:,center_slice_idx));
                
                % Plot target set
                [~, h_target] = contour(xs1_deg, xs2_deg, target_slice, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                hold on;
                
                % Create color map for different steering rate values
                dv_colors = jet(length(dvmax_values));
                
                % Plot BRS boundary for each steering rate
                h_brs = zeros(length(dvmax_values), 1);
                for d_idx = 1:length(dvmax_values)
                    % Extract BRS slice
                    brs_slice = squeeze(all_data{v_idx, d_idx}(:,:,center_slice_idx));
                    
                    [~, h_brs(d_idx)] = contour(xs1_deg, xs2_deg, brs_slice, [0 0], ...
                        'LineWidth', 2, 'Color', dv_colors(d_idx,:));
                end
                
                % Create legend
                legend_entries = cell(length(dvmax_values) + 1, 1);
                for d_idx = 1:length(dvmax_values)
                    legend_entries{d_idx} = sprintf('dv_max = %.0f°/s', dvmax_values(d_idx)*180/pi);
                end
                legend_entries{end} = 'Target Set';
                
                % Add labels, title and legend
                grid on;
                xlabel('Yaw Rate (degrees/s)', 'FontSize', 12);
                ylabel('Sideslip Angle (degrees)', 'FontSize', 12);
                title(sprintf('BRS Comparison for Different Steering Rate Limits (v = %d m/s, δ = %.1f°)', ...
                      velocities(v_idx), delta_val*180/pi), 'FontSize', 14, 'FontWeight', 'bold');
                legend([h_brs; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
                
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
        end
        
        % Comparison of different velocities (if applicable)
        if length(velocities) > 1
            % Compare BRS for different velocities with the same steering rate
            for d_idx = 1:length(dvmax_values)
                figure('Name', sprintf('Velocity Comparison - dvmax=%.1f°/s, delta=%.1f°', ...
                    dvmax_values(d_idx)*180/pi, delta_val*180/pi));
                clf;
                
                % Create a larger figure
                set(gcf, 'Position', [100, 100, 800, 600]);
                
                % Convert grid values from radians to degrees for plotting
                xs1_deg = g.xs{1}(:,:,1) * 180/pi;
                xs2_deg = g.xs{2}(:,:,1) * 180/pi;
                
                % Calculate axis limits
                x_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
                y_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
                
                % Extract target set slice
                target_slice = squeeze(data0(:,:,center_slice_idx));
                
                % Plot target set
                [~, h_target] = contour(xs1_deg, xs2_deg, target_slice, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                hold on;
                
                % Create color map for different velocities
                vel_colors = jet(length(velocities));
                
                % Plot BRS boundary for each velocity
                h_brs = zeros(length(velocities), 1);
                for v_idx = 1:length(velocities)
                    % Extract BRS slice
                    brs_slice = squeeze(all_data{v_idx, d_idx}(:,:,center_slice_idx));
                    
                    [~, h_brs(v_idx)] = contour(xs1_deg, xs2_deg, brs_slice, [0 0], ...
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
                xlabel('Yaw Rate (degrees/s)', 'FontSize', 12);
                ylabel('Sideslip Angle (degrees)', 'FontSize', 12);
                title(sprintf('BRS Comparison for Different Velocities (Max Steering Rate = %.1f°/s, δ = %.1f°)', ...
                      dvmax_values(d_idx)*180/pi, delta_val*180/pi), 'FontSize', 14, 'FontWeight', 'bold');
                legend([h_brs; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
                
                % Add a better grid
                grid minor;
                
                % Set axis limits
                xlim(x_limits);
                ylim(y_limits);
                
                % Save figure if enabled
                if opts.saveFigs
                    fig_filename = sprintf('velocity_comparison_dvmax%.0f_delta%.0f.%s', ...
                        dvmax_values(d_idx)*180/pi, delta_val*180/pi, opts.figFormat);
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                    fprintf('Saved velocity comparison plot to %s\n', fullfile(fig_folder, fig_filename));
                end
            end
        end
        
        % Generate delta slices comparison for the first velocity and steering rate
        for v_idx = 1:length(velocities)
            for d_idx = 1:length(dvmax_values)
                % Get the BRS data
                current_brs = all_data{v_idx, d_idx};
                
                % Select a few delta slices to compare
                num_slices = 5;
                delta_indices = round(linspace(1, size(g.xs{3}, 3), num_slices));
                
                figure('Name', sprintf('Delta Slice Comparison - v=%d, dvmax=%.1f°/s', ...
                    velocities(v_idx), dvmax_values(d_idx)*180/pi));
                clf;
                
                % Create a larger figure
                set(gcf, 'Position', [100, 100, 800, 600]);
                
                % Convert grid values from radians to degrees for plotting
                xs1_deg = g.xs{1}(:,:,1) * 180/pi;
                xs2_deg = g.xs{2}(:,:,1) * 180/pi;
                
                % Calculate axis limits
                x_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
                y_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
                
                % Plot target set (use middle slice as reference)
                target_slice = squeeze(data0(:,:,1));
                [~, h_target] = contour(xs1_deg, xs2_deg, target_slice, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                hold on;
                
                % Create color map for different delta slices
                delta_colors = jet(length(delta_indices));

                % Create legend
                legend_entries = cell(length(delta_indices) + 1, 1);
                
                % Plot BRS boundary for each delta slice
                h_brs = zeros(length(delta_indices), 1);
                for s = 1:length(delta_indices)
                    slice_idx = delta_indices(s);
                    delta_val = delta_values(slice_idx);
                    
                    % Extract BRS slice
                    brs_slice = squeeze(current_brs(:,:,slice_idx));
                    
                    [~, h_brs(s)] = contour(xs1_deg, xs2_deg, brs_slice, [0 0], ...
                        'LineWidth', 2, 'Color', delta_colors(s,:));

                    % Create legend entry
                    legend_entries{s} = sprintf('δ = %.1f°', delta_val*180/pi);
                end
                
                
                legend_entries{end} = 'Target Set';
                
                % Add labels, title and legend
                grid on;
                xlabel('Yaw Rate (degrees/s)', 'FontSize', 12);
                ylabel('Sideslip Angle (degrees)', 'FontSize', 12);
                title(sprintf('BRS Comparison for Different Steering Angles (v = %d m/s, Max Steering Rate = %.1f°/s)', ...
                      velocities(v_idx), dvmax_values(d_idx)*180/pi), 'FontSize', 14, 'FontWeight', 'bold');
                legend([h_brs; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
                
                % Add a better grid
                grid minor;
                
                % Set axis limits
                xlim(x_limits);
                ylim(y_limits);
                
                % Save figure if enabled
                if opts.saveFigs
                    fig_filename = sprintf('delta_comparison_v%d_dvmax%.0f.%s', ...
                        velocities(v_idx), dvmax_values(d_idx)*180/pi, opts.figFormat);
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                    fprintf('Saved delta comparison plot to %s\n', fullfile(fig_folder, fig_filename));
                end
                
                % Only create this visualization for the first combination to avoid too many plots
                break;
            end
            break;
        end
    end
end

% Helper function to extract costates (gradients) from a 2D slice
function derivs = extract2DCostates(g, data_slice)
    % Initialize derivatives
    derivs = cell(2, 1);
    
    % Calculate derivatives using central differences
    % For dimension 1 (gamma)
    deriv_x = zeros(size(data_slice));
    deriv_x(2:end-1,:) = (data_slice(3:end,:) - data_slice(1:end-2,:)) / (2*g.dx(1));
    deriv_x(1,:) = (data_slice(2,:) - data_slice(1,:)) / g.dx(1);
    deriv_x(end,:) = (data_slice(end,:) - data_slice(end-1,:)) / g.dx(1);
    
    % For dimension 2 (beta)
    deriv_y = zeros(size(data_slice));
    deriv_y(:,2:end-1) = (data_slice(:,3:end) - data_slice(:,1:end-2)) / (2*g.dx(2));
    deriv_y(:,1) = (data_slice(:,2) - data_slice(:,1)) / g.dx(2);
    deriv_y(:,end) = (data_slice(:,end) - data_slice(:,end-1)) / g.dx(2);
    
    derivs{1} = deriv_x;
    derivs{2} = deriv_y;
end