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
         
        case 'tire'
            generate_tire_curves();
            
        otherwise
            warning('Unknown plot type: %s', current_plot_type);
    end
end

disp('Visualization complete!');

%% Nested function to generate interactive tire force curve visualization
function generate_tire_curves()
    disp('Generating interactive tire force curve visualization...');
    
    % Loop through velocities and control limits
    for v_idx = 1:length(velocities)
        for m_idx = 1:length(mzmax_values)
            % Create a figure with specified layout
            fig = figure('Name', sprintf('Tire Force Analysis - v=%d, Mz=%d', velocities(v_idx), mzmax_values(m_idx)), ...
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
            
            %% Setup vehicle parameters from BRS data
            % Get vehicle parameters from base_params
            m = base_params(1);      % Vehicle mass
            vx = velocities(v_idx);  % Current longitudinal velocity
            Lf = base_params(3);     % Distance from CG to front axle
            Lr = base_params(4);     % Distance from CG to rear axle
            Iz = base_params(5);     % Yaw moment of inertia
            mu = base_params(6);     % Friction coefficient
            Cf = base_params(9);     % Front tire cornering stiffness
            Cr = base_params(10);    % Rear tire cornering stiffness
            
            % Create a NonlinearBicycle model to use its tire force calculation
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
            
            %% Plot the BRS with current slice
            axes(left_panel);
            
            % Convert grid values from radians to degrees for plotting
            xs1_deg = g.xs{2} * 180/pi;  % Convert sideslip angle to degrees
            xs2_deg = g.xs{1} * 180/pi;  % Convert yaw rate to degrees
            
            % Plot the BRS boundary
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
            title('BRS Boundary - Click to Analyze', 'FontSize', 14);
            legend([h_brs, h_target], 'BRS Boundary', 'Target Set', 'Location', 'best');
            
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
            front_sat_y_limits = get(gca, 'YLim');
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
            rear_sat_y_limits = get(gca, 'YLim');
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
            set(left_panel, 'ButtonDownFcn', @(src, event) analyzePoint(src, event, fig));
            
            % Add title to the figure
            sgtitle(sprintf('Tire Force Analysis for v = %d m/s, Mzmax = %d N·m', ...
                   velocities(v_idx), mzmax_values(m_idx)), 'FontSize', 14, 'FontWeight', 'bold');
            
            % Save figure if enabled
            if opts.saveFigs
                fig_filename = sprintf('tire_curves_v%d_mz%d.%s', ...
                    velocities(v_idx), mzmax_values(m_idx), opts.figFormat);
                saveas(fig, fullfile(fig_folder, fig_filename));
                fprintf('Saved tire curve plot to %s\n', fullfile(fig_folder, fig_filename));
            end
        end
    end
end


%% Nested function to generate derivative plot

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
            
               % Add BRS boundary
               [~, h_brs] = contour(xs1_deg, xs2_deg, current_brs, [0 0], 'LineWidth', 2, 'Color', 'k');
            
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
                magnitudes = log10(sqrt(interp_gamma_deriv.^2 + interp_beta_deriv.^2));
                max_mag = max(magnitudes(:));
                norm_factor = 1 / max_mag;
                
                norm_gamma_deriv = interp_gamma_deriv * norm_factor;
                norm_beta_deriv = interp_beta_deriv * norm_factor;
                
                % Create vector field
                h_quiver = quiver(X, Y, -norm_beta_deriv, -norm_gamma_deriv, 0.5, 'k');
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

% Helper function to calculate tire forces (similar to the one in NonlinearBicycle)
function Fy = calculateTireForce(alpha, C, Fz, mu)
    % Threshold for linear region
    threshold = atan(3*mu*Fz/C);
    
    if abs(alpha) < threshold
        % Linear region with smoothing toward saturation
        Fy = -(-C * alpha + C^2/(3*mu*Fz) * abs(alpha) * alpha - ...
             (1/3) * C^3/(3*mu*Fz)^2 * alpha^3);
    else
        % Saturation region
        Fy = mu*Fz * sign(alpha);
    end
end

% Callback function for mouse clicks on the BRS plot
function analyzePoint(src, event, fig_handle)
    % Get the clicked point in axis coordinates
    point = get(src, 'CurrentPoint');
    x_click = point(1, 1); % Sideslip angle in degrees
    y_click = point(1, 2); % Yaw rate in degrees
    
    % Convert to radians for calculations
    beta = x_click * pi/180;
    gamma = y_click * pi/180;
    
    % Get stored data
    data = get(fig_handle, 'UserData');
    
    % Update point marker on BRS plot
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
                       '  Sideslip Angle (β): %.2f°\n', ...
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