function visualize_brs_results(result_folder, varargin)
% VISUALIZE_BRS_RESULTS Creates visualizations from saved BRS computation results
%
% Inputs:
%   result_folder - Path to the folder containing BRS computation results
%   varargin      - Optional parameter-value pairs:
%                   'plotType'   - Types of plots to generate (cell array)
%                                 Options: 'control', 'detailed', 'comparison', 
%                                 'velocity_stack', 'derivative'
%                                 (default: {'control', 'detailed'})
%                   'saveFigs'   - Whether to save figures (default: true)
%                   'figFormat'  - Format to save figures in (default: 'png')
%
% Example:
%   visualize_brs_results('brs_results_20250325_123456_vx30-50_mz5000-10000');
%   visualize_brs_results('brs_results_folder', 'plotType', {'derivative'});

%% Parse inputs
p = inputParser;
p.addRequired('result_folder', @ischar);
p.addParameter('plotType', {'control', 'detailed'}, @iscell);
p.addParameter('saveFigs', true, @islogical);
p.addParameter('figFormat', 'png', @ischar);

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
     'velocities', 'mzmax_values', 'tau', 'base_params');

%% Load simulation parameters
sim_params_file = fullfile(result_folder, 'sim_params.mat');
if exist(sim_params_file, 'file')
    load(sim_params_file, 'sim_params');
    disp('Loaded simulation parameters.');
else
    warning('Simulation parameters file not found. Using defaults from combined results.');
    sim_params = struct();
    sim_params.velocities = velocities;
    sim_params.mzmax_values = mzmax_values;
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
        case 'control'
            generate_control_plots();
            
        case 'detailed'
            generate_detailed_plots();
            
        case 'comparison'
            generate_comparison_plots();

        case 'velocity_stack'
            generate_3d_velocity_stack();
            
        case 'derivative'
            generate_derivative_plots();
            
        otherwise
            warning('Unknown plot type: %s', current_plot_type);
    end
end

disp('Visualization complete!');

%% Nested function to generate derivative plots
    function generate_derivative_plots()
        disp('Generating derivative plots...');
        
        % Loop through velocities and control limits
        for v_idx = 1:length(velocities)
            for m_idx = 1:length(mzmax_values)
                figure('Name', sprintf('State Derivatives - v=%d, Mz=%d', velocities(v_idx), mzmax_values(m_idx)));
                clf;
                
                % Create a larger figure
                set(gcf, 'Position', [100, 100, 1000, 800]);
                
                % Extract data for the current setup
                current_brs = all_data{v_idx, m_idx};
                current_control = control_data{v_idx, m_idx};
                
                % Convert grid values from radians to degrees for plotting
                xs1_deg = g.xs{2} * 180/pi;  % Convert sideslip angle to degrees
                xs2_deg = g.xs{1} * 180/pi;  % Convert yaw rate to degrees
                
                % Compute gradients of the value function
                derivs = computeGradients(g, current_brs);
                
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
                
                % Add BRS boundary
                [~, h_brs] = contour(xs1_deg, xs2_deg, current_brs, [0 0], 'LineWidth', 2, 'Color', 'k');
                
                % Add zero-crossing contour for the derivative
                [~, h_zero] = contour(xs1_deg, xs2_deg, beta_derivative, [0 0], 'LineWidth', 1.5, 'Color', 'r', 'LineStyle', '--');
                
                title('Derivative w.r.t. Sideslip Angle (β)', 'FontSize', 12);
                xlabel('Sideslip Angle (degrees)', 'FontSize', 10);
                ylabel('Yaw Rate (degrees/s)', 'FontSize', 10);
                legend([h_brs, h_zero], 'BRS Boundary', 'dV/dβ = 0', 'Location', 'best');
                grid on;
                
                % 2. Plot derivative with respect to yaw rate (gamma)
                subplot(2, 2, 2);
                gamma_deriv_plot = pcolor(xs1_deg, xs2_deg, gamma_derivative);
                gamma_deriv_plot.EdgeColor = 'none';
                colormap(gca, 'parula');
                cb = colorbar;
                title(cb, 'dV/dγ');
                hold on;
                
                % Add BRS boundary
                [~, h_brs] = contour(xs1_deg, xs2_deg, current_brs, [0 0], 'LineWidth', 2, 'Color', 'k');
                
                % Add zero-crossing contour for the derivative
                [~, h_zero] = contour(xs1_deg, xs2_deg, gamma_derivative, [0 0], 'LineWidth', 1.5, 'Color', 'r', 'LineStyle', '--');
                
                title('Derivative w.r.t. Yaw Rate (γ)', 'FontSize', 12);
                xlabel('Sideslip Angle (degrees)', 'FontSize', 10);
                ylabel('Yaw Rate (degrees/s)', 'FontSize', 10);
                legend([h_brs, h_zero], 'BRS Boundary', 'dV/dγ = 0', 'Location', 'best');
                grid on;
                
                % 3. Plot derivative magnitude
                subplot(2, 2, 3);
                % Calculate gradient magnitude (Euclidean norm)
                gradient_magnitude = sqrt(gamma_derivative.^2 + beta_derivative.^2);
                
                magnitude_plot = pcolor(xs1_deg, xs2_deg, gradient_magnitude);
                magnitude_plot.EdgeColor = 'none';
                colormap(gca, 'hot');
                cb = colorbar;
                title(cb, '||∇V||');
                hold on;
                
                % Add BRS boundary
                [~, h_brs] = contour(xs1_deg, xs2_deg, current_brs, [0 0], 'LineWidth', 2, 'Color', 'k');
                
                title('Gradient Magnitude ||∇V||', 'FontSize', 12);
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
                h_quiver = quiver(X, Y, norm_beta_deriv, norm_gamma_deriv, 0.5, 'k');
                hold on;
                
                % Add BRS boundary
                [~, h_brs] = contour(xs1_deg, xs2_deg, current_brs, [0 0], 'LineWidth', 2, 'Color', 'k');
                
                % Add target set
                [~, h_target] = contour(xs1_deg, xs2_deg, data0, [0 0], 'LineWidth', 1.5, 'Color', 'g', 'LineStyle', '--');
                
                title('Gradient Vector Field ∇V', 'FontSize', 12);
                xlabel('Sideslip Angle (degrees)', 'FontSize', 10);
                ylabel('Yaw Rate (degrees/s)', 'FontSize', 10);
                legend([h_brs, h_target], 'BRS Boundary', 'Target Set', 'Location', 'best');
                grid on;
                
                % Add a main title for the entire figure
                sgtitle(sprintf('Gradient Analysis for v = %d m/s, Mzmax = %d N·m', ...
                    velocities(v_idx), mzmax_values(m_idx)), 'FontSize', 14, 'FontWeight', 'bold');
                
                % Save figure if enabled
                if opts.saveFigs
                    fig_filename = sprintf('derivative_v%d_mz%d.%s', ...
                        velocities(v_idx), mzmax_values(m_idx), opts.figFormat);
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                    fprintf('Saved derivative plot to %s\n', fullfile(fig_folder, fig_filename));
                end
            end
        end
    end

%% Nested function to generate basic control plots
    function generate_control_plots()
        disp('Generating control plots...');
        
        for v_idx = 1:length(velocities)
            for m_idx = 1:length(mzmax_values)
                figure('Name', sprintf('Control - v=%d, Mz=%d', velocities(v_idx), mzmax_values(m_idx)));
                clf;
                
                % Create a larger figure
                set(gcf, 'Position', [100, 100, 800, 600]);
                
                % Convert grid values from radians to degrees for plotting
                xs1_deg = g.xs{2} * 180/pi;  % Convert sideslip angle to degrees
                xs2_deg = g.xs{1} * 180/pi;  % Convert yaw rate to degrees
                
                % Get the BRS data and control data for this combination
                current_brs = all_data{v_idx, m_idx};
                current_control = control_data{v_idx, m_idx};
                
                % Plot control input with colored background
                control_plot = pcolor(xs1_deg, xs2_deg, current_control);
                control_plot.EdgeColor = 'none';
                colormap(custom_cmap);
                
                % Add a colorbar
                cb = colorbar;
                title(cb, 'Yaw Moment (N·m)');
                
                % Set the colorbar limits to match the Mzmax values
                caxis([-mzmax_values(m_idx), mzmax_values(m_idx)]);
                
                hold on;
                
                % Plot the BRS boundary
                [~, h_brs] = contour(xs1_deg, xs2_deg, current_brs, [0 0], 'LineWidth', 2, 'Color', 'k');
                
                % Plot the target set
                [~, h_target] = contour(xs1_deg, xs2_deg, data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                
                % Add grid, labels, and legend
                grid on;
                xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
                ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
                title(sprintf('BRS and Optimal Control for v = %d m/s, Mzmax = %d N·m', ...
                      velocities(v_idx), mzmax_values(m_idx)), 'FontSize', 14, 'FontWeight', 'bold');
                legend([h_brs, h_target], 'BRS Boundary', 'Target Set', 'Location', 'best', 'FontSize', 10);
                
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
                    fig_filename = sprintf('control_v%d_mz%d.%s', ...
                        velocities(v_idx), mzmax_values(m_idx), opts.figFormat);
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                    fprintf('Saved control plot to %s\n', fullfile(fig_folder, fig_filename));
                end
            end
        end
    end

%% Nested function to generate detailed plots with control strategies
    function generate_detailed_plots()
        disp('Generating detailed control analysis plots...');
        
        for v_idx = 1:length(velocities)
            for m_idx = 1:length(mzmax_values)
                figure('Name', sprintf('Detailed - v=%d, Mz=%d', velocities(v_idx), mzmax_values(m_idx)));
                clf;
                
                % Create a larger figure
                set(gcf, 'Position', [100, 100, 900, 700]);
                
                % Convert grid values from radians to degrees for plotting
                xs1_deg = g.xs{2} * 180/pi;  % Convert sideslip angle to degrees
                xs2_deg = g.xs{1} * 180/pi;  % Convert yaw rate to degrees
                
                % Calculate axis limits
                x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
                y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
                
                % Get the control data for this combination
                current_control = control_data{v_idx, m_idx};
                current_brs = all_data{v_idx, m_idx};
                
                % 1. Control Input Visualization
                subplot(2, 2, 1);
                
                % Create a pcolor plot with the custom colormap
                control_plot = pcolor(xs1_deg, xs2_deg, current_control);
                control_plot.EdgeColor = 'none';
                colormap(gca, custom_cmap);
                caxis([-mzmax_values(m_idx), mzmax_values(m_idx)]);
                
                % Add contour of BRS boundary
                hold on;
                [~, h_brs] = contour(xs1_deg, xs2_deg, current_brs, [0 0], 'LineWidth', 2, 'Color', 'k');
                
                % Add colorbar and labels
                cb = colorbar;
                title(cb, 'Yaw Moment (N·m)');
                
                title('Optimal Control Input', 'FontSize', 12);
                ylabel('Sideslip Angle (deg)', 'FontSize', 10);
                xlabel('Yaw Rate (deg/s)', 'FontSize', 10);
                xlim(x_limits);
                ylim(y_limits);
                grid on;
                
                % 2. BRS Boundary with Control Switching Surface
                subplot(2, 2, 2);
                
                % Plot BRS boundary
                [~, h_brs] = contour(xs1_deg, xs2_deg, current_brs, [0 0], 'LineWidth', 2, 'Color', 'k');
                hold on;
                
                % Plot target set
                [~, h_target] = contour(xs1_deg, xs2_deg, data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                
                % Find and plot the control switching surface
                % Switching occurs when the sign of the gradient of value function changes
                derivs = extractCostates(g, current_brs);
                switching_condition = derivs{2};  % The derivative itself, not just the sign
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
                surf(xs1_deg, xs2_deg, current_brs, 'EdgeColor', 'none');
                colormap(gca, parula);
                
                % Adjust view angle
                view([-30, 30]);
                
                % Add the zero level set (BRS boundary)
                hold on;
                [~, h_brs] = contour3(xs1_deg, xs2_deg, current_brs, [0 0], 'LineWidth', 2, 'Color', 'r');
                
                % Add colorbar and labels
                cb = colorbar;
                title(cb, 'Value Function');
                
                title('Value Function Surface', 'FontSize', 12);
                xlabel('Sideslip Angle (deg)', 'FontSize', 10);
                ylabel('Yaw Rate (deg/s)', 'FontSize', 10);
                zlabel('Value', 'FontSize', 10);
                grid on;
                
                % Add a main title for the entire figure
                sgtitle(sprintf('Detailed Control Analysis (v = %d m/s, Mzmax = %d N·m)', ...
                        velocities(v_idx), mzmax_values(m_idx)), 'FontSize', 14, 'FontWeight', 'bold');
                
                % Save figure if enabled
                if opts.saveFigs
                    fig_filename = sprintf('detailed_v%d_mz%d.%s', ...
                        velocities(v_idx), mzmax_values(m_idx), opts.figFormat);
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                    fprintf('Saved detailed plot to %s\n', fullfile(fig_folder, fig_filename));
                end
            end
        end
    end

%% Nested function to generate 3D plots showing BRSs for different speeds
    function generate_3d_velocity_stack()
        disp('Generating velocity stack...');
        
        % Comparison of different velocities (if applicable)
        if length(velocities) > 1
            % Compare BRS for different velocities with the same Mzmax
            for m_idx = 1:length(mzmax_values)
                figure('Name', sprintf('Velocity Comparison - Mz=%d', mzmax_values(m_idx)));
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
                
                % Plot BRS boundary for each velocity
                h_brs = zeros(length(velocities), 1);
                for v_idx = 1:length(velocities)
                    [~, h_brs(v_idx)] = contour3(xs1_deg, xs2_deg, ...
                        (all_data{v_idx, m_idx}) + velocities(v_idx), ...
                        [velocities(v_idx) velocities(v_idx)], ...
                        'LineWidth', 2, 'Color', vel_colors(v_idx,:));
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
                
                % Add labels, title and legend
                grid on;
                xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
                ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
                zlabel('Longitudinal Velocirt (m/s)', 'FontSize', 12);
                title(sprintf('BRS Comparison for Different Velocities (Mzmax = %d N·m)', ...
                      mzmax_values(m_idx)), 'FontSize', 14, 'FontWeight', 'bold');
                % legend([h_brs; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
                
                % Add a better grid
                grid minor;
                
                % Set axis limits
                xlim(x_limits);
                ylim(y_limits);
                zlim([0 50])

                view([-30 30]);
                
                % Save figure if enabled
                if opts.saveFigs
                    fig_filename = sprintf('velocity_comparison_mz%d.%s', ...
                        mzmax_values(m_idx), opts.figFormat);
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                    fprintf('Saved velocity comparison plot to %s\n', fullfile(fig_folder, fig_filename));
                end
            end
        end
    end

%% Nested function to generate comparison plots across different parameters
    function generate_comparison_plots()
        disp('Generating comparison plots...');
        
        % Comparison of different velocities (if applicable)
        if length(velocities) > 1
            % Compare BRS for different velocities with the same Mzmax
            for m_idx = 1:length(mzmax_values)
                figure('Name', sprintf('Velocity Comparison - Mz=%d', mzmax_values(m_idx)));
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
                
                % Plot BRS boundary for each velocity
                h_brs = zeros(length(velocities), 1);
                for v_idx = 1:length(velocities)
                    [~, h_brs(v_idx)] = contour(xs1_deg, xs2_deg, all_data{v_idx, m_idx}, [0 0], ...
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
                title(sprintf('BRS Comparison for Different Velocities (Mzmax = %d N·m)', ...
                      mzmax_values(m_idx)), 'FontSize', 14, 'FontWeight', 'bold');
                legend([h_brs; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
                
                % Add a better grid
                grid minor;
                
                % Set axis limits
                xlim(x_limits);
                ylim(y_limits);
                
                % Save figure if enabled
                if opts.saveFigs
                    fig_filename = sprintf('velocity_comparison_mz%d.%s', ...
                        mzmax_values(m_idx), opts.figFormat);
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                    fprintf('Saved velocity comparison plot to %s\n', fullfile(fig_folder, fig_filename));
                end
            end
        end
        
        % Comparison of different Mzmax values (if applicable)
        if length(mzmax_values) > 1
            % Compare BRS for different Mzmax with the same velocity
            for v_idx = 1:length(velocities)
                figure('Name', sprintf('Mzmax Comparison - v=%d', velocities(v_idx)));
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
                
                % Create color map for different Mzmax values
                mz_colors = jet(length(mzmax_values));
                
                % Plot BRS boundary for each Mzmax
                h_brs = zeros(length(mzmax_values), 1);
                for m_idx = 1:length(mzmax_values)
                    [~, h_brs(m_idx)] = contour(xs1_deg, xs2_deg, all_data{v_idx, m_idx}, [0 0], ...
                        'LineWidth', 2, 'Color', mz_colors(m_idx,:));
                end
                
                % Create legend
                legend_entries = cell(length(mzmax_values) + 1, 1);
                for m_idx = 1:length(mzmax_values)
                    legend_entries{m_idx} = sprintf('Mzmax = %d N·m', mzmax_values(m_idx));
                end
                legend_entries{end} = 'Target Set';
                
                % Add labels, title and legend
                grid on;
                xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
                ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
                title(sprintf('BRS Comparison for Different Yaw Moment Limits (v = %d m/s)', ...
                      velocities(v_idx)), 'FontSize', 14, 'FontWeight', 'bold');
                legend([h_brs; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
                
                % Add a better grid
                grid minor;
                
                % Set axis limits
                xlim(x_limits);
                ylim(y_limits);
                
                % Save figure if enabled
                if opts.saveFigs
                    fig_filename = sprintf('mzmax_comparison_v%d.%s', ...
                        velocities(v_idx), opts.figFormat);
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                    fprintf('Saved Mzmax comparison plot to %s\n', fullfile(fig_folder, fig_filename));
                end
            end
        end
        
        % Time evolution visualization (for the first velocity and Mzmax combination)
        % Show how the BRS evolves over time

        for v_idx=1:length(velocities)
            for m_idx=1:length(mzmax_values)
                
                if ~isempty(all_data_full) && ~isempty(all_data_full{v_idx, m_idx})
                    figure('Name', 'BRS Time Evolution');
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
                    
                    % Plot BRS boundary for each time point
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
                    title(sprintf('BRS Time Evolution (v = %d m/s, Mzmax = %d N·m)', ...
                          velocities(v_idx), mzmax_values(m_idx)), 'FontSize', 14, 'FontWeight', 'bold');
                    legend([h_time; h_target], legend_entries, 'Location', 'best', 'FontSize', 10);
                    
                    % Add a better grid
                    grid minor;
                    
                    % Set axis limits
                    xlim(x_limits);
                    ylim(y_limits);
                    
                    % Save figure if enabled
                    if opts.saveFigs
                        fig_filename = sprintf('time_evolution_v%d_mz%d.%s', ...
                            velocities(v_idx), mzmax_values(m_idx), opts.figFormat);
                        saveas(gcf, fullfile(fig_folder, fig_filename));
                        fprintf('Saved time evolution plot to %s\n', fullfile(fig_folder, fig_filename));
                    end
                end
            end        
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