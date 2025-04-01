function visualize_brs_frs_comparison(brs_folder, frs_folder, varargin)
% VISUALIZE_BRS_FRS_COMPARISON Compares and visualizes backward and forward reachable sets
%
% Inputs:
%   brs_folder - Path to the folder containing BRS computation results
%   frs_folder - Path to the folder containing FRS computation results
%   varargin   - Optional parameter-value pairs:
%                 'saveFigs'   - Whether to save figures (default: true)
%                 'figFormat'  - Format to save figures in (default: 'png')
%                 'model'      - Model type: '2state' or '3state' (default: '2state')
%
% Example:
%   visualize_brs_frs_comparison('brs_results_folder', 'frs_results_folder');
%   visualize_brs_frs_comparison('brs_folder', 'frs_folder', 'model', '3state');

%% Parse inputs
p = inputParser;
p.addRequired('brs_folder', @ischar);
p.addRequired('frs_folder', @ischar);
p.addParameter('saveFigs', true, @islogical);
p.addParameter('figFormat', 'png', @ischar);
p.addParameter('model', '2state', @ischar); % '2state' or '3state'

p.parse(brs_folder, frs_folder, varargin{:});
opts = p.Results;

%% Check if the folders exist
if ~exist(brs_folder, 'dir')
    error('BRS folder does not exist: %s', brs_folder);
end

if ~exist(frs_folder, 'dir')
    error('FRS folder does not exist: %s', frs_folder);
end

%% Load results
% Load BRS results
brs_file = fullfile(brs_folder, 'brs_combined_results.mat');
if ~exist(brs_file, 'file')
    % Try alternative filename format
    brs_file = fullfile(brs_folder, 'frs_combined_results.mat');
    if ~exist(brs_file, 'file')
        error('Combined results file not found in BRS folder');
    end
end
disp(['Loading BRS results from: ', brs_file]);
brs_data = load(brs_file);

% Load FRS results
frs_file = fullfile(frs_folder, 'frs_combined_results.mat');
if ~exist(frs_file, 'file')
    % Try alternative filename format
    frs_file = fullfile(frs_folder, 'brs_combined_results.mat');
    if ~exist(frs_file, 'file')
        error('Combined results file not found in FRS folder');
    end
end
disp(['Loading FRS results from: ', frs_file]);
frs_data = load(frs_file);

%% Create output folder for figures if saving is enabled
if opts.saveFigs
    comparison_folder = fullfile(pwd, 'comparison_results');
    if ~exist(comparison_folder, 'dir')
        mkdir(comparison_folder);
    end
    
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    fig_folder = fullfile(comparison_folder, ['comparison_' timestamp]);
    
    if ~exist(fig_folder, 'dir')
        mkdir(fig_folder);
        disp(['Created comparison figures directory: ', fig_folder]);
    end
end

%% Check data consistency
% Check if both datasets have the same grid
if ~isequal(brs_data.g.min, frs_data.g.min) || ~isequal(brs_data.g.max, frs_data.g.max) || ~isequal(brs_data.g.N, frs_data.g.N)
    warning('BRS and FRS grids are not identical. This may lead to incorrect comparisons.');
end

%% Generate comparisons based on model type
if strcmp(opts.model, '2state')
    % For 2-state model (NonlinearBicycle with yaw moment control)
    generate_2state_comparisons();
else
    % For 3-state model (NonlinearBicycleSteered with steering rate control)
    generate_3state_comparisons();
end

disp('Comparison complete!');

%% Nested function for 2-state model comparisons
    function generate_2state_comparisons()
        disp('Generating comparisons for 2-state model...');
        
        % Get common parameter sets
        common_velocities = intersect(brs_data.velocities, frs_data.velocities);
        if strcmp(opts.model, '2state')
            common_control_limits = intersect(brs_data.mzmax_values, frs_data.mzmax_values);
            control_param_name = 'mzmax_values';
            control_param_unit = 'N·m';
        else
            common_control_limits = intersect(brs_data.dvmax_values, frs_data.dvmax_values);
            control_param_name = 'dvmax_values';
            control_param_unit = 'rad/s';
        end
        
        if isempty(common_velocities) || isempty(common_control_limits)
            error('No common parameter sets found between BRS and FRS data.');
        end
        
        % Create a comparison plot for each parameter combination
        for v_idx = 1:length(common_velocities)
            velocity = common_velocities(v_idx);
            
            % Find corresponding indices in original data
            brs_v_idx = find(brs_data.velocities == velocity);
            frs_v_idx = find(frs_data.velocities == velocity);
            
            for c_idx = 1:length(common_control_limits)
                control_limit = common_control_limits(c_idx);
                
                % Find corresponding indices in original data
                if strcmp(opts.model, '2state')
                    brs_c_idx = find(brs_data.mzmax_values == control_limit);
                    frs_c_idx = find(frs_data.mzmax_values == control_limit);
                    param_label = sprintf('Mzmax = %d %s', control_limit, control_param_unit);
                else
                    brs_c_idx = find(brs_data.dvmax_values == control_limit);
                    frs_c_idx = find(frs_data.dvmax_values == control_limit);
                    param_label = sprintf('dvmax = %.1f %s', control_limit, control_param_unit);
                end
                
                % Get data for this velocity and control limit
                if strcmp(opts.model, '2state')
                    brs = brs_data.all_data{brs_v_idx, brs_c_idx};
                    frs = frs_data.all_data{frs_v_idx, frs_c_idx};
                else
                    % For 3D data, take the middle slice (along delta dimension)
                    delta_mid_idx = ceil(size(brs_data.all_data{brs_v_idx, brs_c_idx}, 3) / 2);
                    brs = squeeze(brs_data.all_data{brs_v_idx, brs_c_idx}(:,:,delta_mid_idx));
                    frs = squeeze(frs_data.all_data{frs_v_idx, frs_c_idx}(:,:,delta_mid_idx));
                end
                
                % Create figure
                figure('Name', sprintf('BRS-FRS Comparison - v=%d, %s', velocity, param_label));
                set(gcf, 'Position', [100, 100, 1200, 900]);
                
                % Get grid data (convert to degrees for visualization)
                g = brs_data.g;
                xs1_deg = g.xs{1} * 180/pi;  % Gamma (yaw rate) in degrees
                xs2_deg = g.xs{2} * 180/pi;  % Beta (sideslip angle) in degrees
                
                % Calculate axis limits
                x_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];
                y_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];
                
                % 1. Plot BRS, FRS, and their intersection
                subplot(2, 2, 1);
                
                % Plot BRS
                [~, h_brs] = contour(xs1_deg, xs2_deg, brs, [0 0], 'LineWidth', 2, 'Color', 'b');
                hold on;
                
                % Plot FRS
                [~, h_frs] = contour(xs1_deg, xs2_deg, frs, [0 0], 'LineWidth', 2, 'Color', 'r');
                
                % Plot target set
                [~, h_target] = contour(xs1_deg, xs2_deg, brs_data.data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                
                % Compute the intersection of BRS and FRS (where both are <= 0)
                intersection = max(brs, frs);
                [~, h_int] = contour(xs1_deg, xs2_deg, intersection, [0 0], 'LineWidth', 2, 'Color', 'k', 'LineStyle', '-.');
                
                grid on;
                title('BRS, FRS, and Their Intersection', 'FontSize', 14);
                xlabel('Yaw Rate (deg/s)', 'FontSize', 12);
                ylabel('Sideslip Angle (deg)', 'FontSize', 12);
                legend([h_brs, h_frs, h_target, h_int], 'BRS', 'FRS', 'Target Set', 'Intersection', 'Location', 'best');
                xlim(x_limits);
                ylim(y_limits);
                
                % 2. Plot BRS with filled area
                subplot(2, 2, 2);
                
                % Create filled contour for BRS
                contourf(xs1_deg, xs2_deg, brs, [0 0], 'LineWidth', 2);
                colormap(gca, [0.8 0.8 1; 1 1 1]); % Light blue for BRS
                hold on;
                
                % Plot target set
                [~, h_target] = contour(xs1_deg, xs2_deg, brs_data.data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                
                grid on;
                title('Backward Reachable Set (BRS)', 'FontSize', 14);
                xlabel('Yaw Rate (deg/s)', 'FontSize', 12);
                ylabel('Sideslip Angle (deg)', 'FontSize', 12);
                legend(h_target, 'Target Set', 'Location', 'best');
                xlim(x_limits);
                ylim(y_limits);
                
                % 3. Plot FRS with filled area
                subplot(2, 2, 3);
                
                % Create filled contour for FRS
                contourf(xs1_deg, xs2_deg, frs, [0 0], 'LineWidth', 2);
                colormap(gca, [1 0.8 0.8; 1 1 1]); % Light red for FRS
                hold on;
                
                % Plot target set
                [~, h_target] = contour(xs1_deg, xs2_deg, brs_data.data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                
                grid on;
                title('Forward Reachable Set (FRS)', 'FontSize', 14);
                xlabel('Yaw Rate (deg/s)', 'FontSize', 12);
                ylabel('Sideslip Angle (deg)', 'FontSize', 12);
                legend(h_target, 'Target Set', 'Location', 'best');
                xlim(x_limits);
                ylim(y_limits);
                
                % 4. Plot intersection with filled area
                subplot(2, 2, 4);
                
                % Create filled contour for intersection
                contourf(xs1_deg, xs2_deg, intersection, [0 0], 'LineWidth', 2);
                colormap(gca, [0.7 0.9 0.7; 1 1 1]); % Light green for intersection
                hold on;
                
                % Plot target set
                [~, h_target] = contour(xs1_deg, xs2_deg, brs_data.data0, [0 0], 'LineWidth', 2, 'Color', 'g', 'LineStyle', '--');
                
                grid on;
                title('Intersection of BRS and FRS', 'FontSize', 14);
                xlabel('Yaw Rate (deg/s)', 'FontSize', 12);
                ylabel('Sideslip Angle (deg)', 'FontSize', 12);
                legend(h_target, 'Target Set', 'Location', 'best');
                xlim(x_limits);
                ylim(y_limits);
                
                % Add a main title for the entire figure
                sgtitle(sprintf('BRS-FRS Comparison (v = %d m/s, %s)', velocity, param_label), 'FontSize', 16, 'FontWeight', 'bold');
                
                % Save figure if enabled
                if opts.saveFigs
                    if strcmp(opts.model, '2state')
                        fig_filename = sprintf('comparison_v%d_mz%d.%s', velocity, control_limit, opts.figFormat);
                    else
                        fig_filename = sprintf('comparison_v%d_dvmax%.0f.%s', velocity, control_limit*180/pi, opts.figFormat);
                    end
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                    fprintf('Saved comparison to %s\n', fullfile(fig_folder, fig_filename));
                end
                
                % Calculate metrics and display them
                % Calculate areas
                brs_area = sum(brs(:) <= 0) / numel(brs);
                frs_area = sum(frs(:) <= 0) / numel(frs);
                int_area = sum(intersection(:) <= 0) / numel(intersection);
                
                % Calculate Dice coefficient (2*|A∩B|/(|A|+|B|))
                dice = 2 * int_area / (brs_area + frs_area);
                
                fprintf('\nComparison Metrics (v = %d m/s, %s):\n', velocity, param_label);
                fprintf('  BRS relative area: %.2f%%\n', brs_area * 100);
                fprintf('  FRS relative area: %.2f%%\n', frs_area * 100);
                fprintf('  Intersection relative area: %.2f%%\n', int_area * 100);
                fprintf('  Dice coefficient: %.4f\n', dice);
                
                % Create a text summary figure
                figure('Name', sprintf('Metrics - v=%d, %s', velocity, param_label));
                set(gcf, 'Position', [100, 100, 600, 300]);
                
                % Create empty axes
                ax = axes('Position', [0 0 1 1], 'Visible', 'off');
                
                % Create text
                text_str = {
                    sprintf('Comparison Metrics (v = %d m/s, %s):', velocity, param_label),
                    sprintf('BRS relative area: %.2f%%', brs_area * 100),
                    sprintf('FRS relative area: %.2f%%', frs_area * 100),
                    sprintf('Intersection relative area: %.2f%%', int_area * 100),
                    sprintf('Dice coefficient: %.4f', dice)
                };
                
                text(0.1, 0.6, text_str, 'FontSize', 14, 'VerticalAlignment', 'top');
                
                % Save metrics figure if enabled
                if opts.saveFigs
                    if strcmp(opts.model, '2state')
                        fig_filename = sprintf('metrics_v%d_mz%d.%s', velocity, control_limit, opts.figFormat);
                    else
                        fig_filename = sprintf('metrics_v%d_dvmax%.0f.%s', velocity, control_limit*180/pi, opts.figFormat);
                    end
                    saveas(gcf, fullfile(fig_folder, fig_filename));
                end
            end
        end
    end

%% Nested function for 3-state model comparisons
    function generate_3state_comparisons()
        disp('Generating comparisons for 3-state model...');
        
        % Just call the 2-state function - the code handles both models
        generate_2state_comparisons();
    end
end