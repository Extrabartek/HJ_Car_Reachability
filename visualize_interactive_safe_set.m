function visualize_interactive_safe_set(g, safe_set, brs, frs, velocity, control_limit, control_type, brs_tMax, frs_tMax, target_set)
% VISUALIZE_INTERACTIVE_SAFE_SET Interactive visualization for safe set with 2D slices and 3D view
%
% This function creates an interactive visualization with:
% - A slider to control the steering angle for slicing
% - Left panel: 2D slice of the safe set at the selected steering angle
% - Right panel: 3D visualization with a plane showing the slice location
%
% Inputs:
%   g              - Grid structure from reachability analysis
%   safe_set       - Safe set value function (3D)
%   brs            - Backward reachable set value function (3D)
%   frs            - Forward reachable set value function (3D)
%   velocity       - Vehicle velocity (m/s)
%   control_limit  - Control limit (depends on control_type)
%   control_type   - Control type ('dv' for steering rate, 'mz' for yaw moment)
%   brs_tMax       - Maximum time for BRS computation (optional)
%   frs_tMax       - Maximum time for FRS computation (optional)
%   target_set     - Target set used for BRS computation (optional)

% Handle optional time parameters
if nargin < 8
    brs_tMax = [];
end
if nargin < 9
    frs_tMax = [];
end
if nargin < 10
    target_set = [];
end

% Create the main figure
fig = figure('Name', 'Interactive Safe Set Visualization', ...
            'Position', [100, 100, 1400, 800], ...
            'Color', 'white');

% Get 3rd dimension values (steering angle)
delta_values = g.xs{3}(1, 1, :);
num_delta = length(delta_values);

% Initial slice index (middle)
current_slice_idx = ceil(num_delta/2);

% Convert grid values from radians to degrees for visualization
[beta_grid, gamma_grid, delta_grid] = meshgrid(...
    g.xs{2}(1,:,1) * 180/pi,...  % beta (sideslip angle)
    g.xs{1}(:,1,1) * 180/pi,...  % gamma (yaw rate)
    g.xs{3}(1,1,:) * 180/pi);    % delta (steering angle)

% Create 2D axes for slice view (left panel)
ax_2d = subplot(1, 2, 1);
set(ax_2d, 'Position', [0.05, 0.2, 0.42, 0.7]);

% Create 3D axes for isosurface view (right panel)
ax_3d = subplot(1, 2, 2);
set(ax_3d, 'Position', [0.55, 0.2, 0.42, 0.7]);

% Create slider for steering angle
slider = uicontrol('Style', 'slider', ...
                   'Position', [120, 40, 260, 20], ...
                   'Min', 1, 'Max', num_delta, ...
                   'Value', current_slice_idx, ...
                   'SliderStep', [1/(num_delta-1), 10/(num_delta-1)]);
               
% Add text display for current steering angle
text_display = uicontrol('Style', 'text', ...
                         'Position', [slider.Position(1) + slider.Position(3) + 10, slider.Position(2), 120, 25], ...
                         'String', sprintf('δ = %.1f°', delta_values(current_slice_idx)*180/pi), ...
                         'HorizontalAlignment', 'left', ...
                         'FontSize', 12);

% Initialize slice visualization (left panel)
axes(ax_2d);
[h_brs, h_frs, h_safe, h_safe_region, h_target_2d] = initialize_2d_slice(current_slice_idx);

% Initialize 3D visualization (right panel)
axes(ax_3d);
[h_brs_3d, h_frs_3d, h_safe_3d, h_slice_plane, h_target_3d] = initialize_3d_view();

% Add title based on parameters
if strcmp(control_type, 'dv')
    fig_title = sprintf('Safe Set Visualization (v = %d m/s, dvmax = %.1f°/s', ...
            velocity, control_limit*180/pi);
else
    fig_title = sprintf('Safe Set Visualization (v = %d m/s, Mzmax = %d N·m', ...
            velocity, control_limit);
end

% Add time information to title if available
if ~isempty(brs_tMax) && ~isempty(frs_tMax)
    fig_title = [fig_title, sprintf(', BRS tMax = %.1f s, FRS tMax = %.1f s)', brs_tMax, frs_tMax)];
elseif ~isempty(brs_tMax)
    fig_title = [fig_title, sprintf(', BRS tMax = %.1f s)', brs_tMax)];
elseif ~isempty(frs_tMax)
    fig_title = [fig_title, sprintf(', FRS tMax = %.1f s)', frs_tMax)];
else
    fig_title = [fig_title, ')'];
end

sgtitle(fig_title, 'FontSize', 16, 'FontWeight', 'bold');

% Add toggle buttons for 3D visualization
btn_width = 100;
btn_height = 30;
spacing = 20;

% Position calculation for centered buttons
fig_width = fig.Position(3);
total_btn_width = 4*btn_width + 3*spacing;  % Now 4 buttons
left_pos = (fig_width - total_btn_width) / 2;

% BRS Toggle Button
brs_btn = uicontrol('Style', 'togglebutton', ...
                    'String', 'BRS (Blue)', ...
                    'Position', [left_pos, 20, btn_width, btn_height], ...
                    'Value', 1, ...  % Initially on
                    'BackgroundColor', [0.8 0.8 1], ... % Light blue
                    'FontWeight', 'bold');
                
% FRS Toggle Button
frs_btn = uicontrol('Style', 'togglebutton', ...
                    'String', 'FRS (Red)', ...
                    'Position', [left_pos + btn_width + spacing, 20, btn_width, btn_height], ...
                    'Value', 1, ...  % Initially on
                    'BackgroundColor', [1 0.8 0.8], ... % Light red
                    'FontWeight', 'bold');
                
% Safe Set Toggle Button
safe_btn = uicontrol('Style', 'togglebutton', ...
                    'String', 'Safe Set (Green)', ...
                    'Position', [left_pos + 2*(btn_width + spacing), 20, btn_width, btn_height], ...
                    'Value', 1, ...  % Initially on
                    'BackgroundColor', [0.8 1 0.8], ... % Light green
                    'FontWeight', 'bold');
                
% Target Set Toggle Button
target_btn = uicontrol('Style', 'togglebutton', ...
                    'String', 'Target (Yellow)', ...
                    'Position', [left_pos + 3*(btn_width + spacing), 20, btn_width, btn_height], ...
                    'Value', 1, ...  % Initially on
                    'BackgroundColor', [1 1 0.8], ... % Light yellow
                    'FontWeight', 'bold');

% Set up slider callback
slider.Callback = @update_slice;

% Store visualization handles and data for callback
slider_data = struct();
slider_data.g = g;
slider_data.safe_set = safe_set;
slider_data.brs = brs;
slider_data.frs = frs;
slider_data.target_set = target_set;
slider_data.beta_grid = beta_grid;
slider_data.gamma_grid = gamma_grid;
slider_data.delta_grid = delta_grid;
slider_data.delta_values = delta_values;
slider_data.h_brs = h_brs;
slider_data.h_frs = h_frs;
slider_data.h_safe = h_safe;
slider_data.h_safe_region = h_safe_region;
slider_data.h_target_2d = h_target_2d;
slider_data.h_brs_3d = h_brs_3d;
slider_data.h_frs_3d = h_frs_3d;
slider_data.h_safe_3d = h_safe_3d;
slider_data.h_target_3d = h_target_3d;
slider_data.h_slice_plane = h_slice_plane;
slider_data.text_display = text_display;
slider_data.ax_2d = ax_2d;
slider_data.ax_3d = ax_3d;
slider_data.brs_btn = brs_btn;
slider_data.frs_btn = frs_btn;
slider_data.safe_btn = safe_btn;
slider_data.target_btn = target_btn;

% Set callbacks for toggle buttons
brs_btn.Callback = @(src, ~) toggleVisibility(src, h_brs_3d, 'BRS');
frs_btn.Callback = @(src, ~) toggleVisibility(src, h_frs_3d, 'FRS');
safe_btn.Callback = @(src, ~) toggleVisibility(src, h_safe_3d, 'Safe Set');
target_btn.Callback = @(src, ~) toggleVisibility(src, h_target_3d, 'Target Set');

% Store slider data in the figure
set(fig, 'UserData', slider_data);

%% Nested function to initialize 2D slice visualization
    function [h_brs, h_frs, h_safe, h_safe_region, h_target] = initialize_2d_slice(slice_idx)
        % Extract 2D slices for the selected steering angle
        brs_slice = squeeze(brs(:,:,slice_idx));
        frs_slice = squeeze(frs(:,:,slice_idx));
        safe_slice = squeeze(safe_set(:,:,slice_idx));
        
        % Get target slice (centered slice if target set available)
        h_target = [];
        if ~isempty(target_set)
            target_center_idx = ceil(size(target_set, 3) / 2);
            target_slice = squeeze(target_set(:,:,target_center_idx));
        end
        
        % Get 2D grid coordinates
        xs1_deg = g.xs{1}(:,:,1) * 180/pi;  % yaw rate (gamma)
        xs2_deg = g.xs{2}(:,:,1) * 180/pi;  % sideslip angle (beta)
        
        hold(ax_2d, 'on');
        
        % Plot the set boundaries
        [~, h_brs] = contour(xs2_deg, xs1_deg, brs_slice, [0 0], 'LineWidth', 2, 'Color', 'b');
        [~, h_frs] = contour(xs2_deg, xs1_deg, frs_slice, [0 0], 'LineWidth', 2, 'Color', 'r');
        [~, h_safe] = contour(xs2_deg, xs1_deg, safe_slice, [0 0], 'LineWidth', 3, 'Color', 'k');
        
        % Plot target set if available
        if ~isempty(target_set)
            [~, h_target] = contour(xs2_deg, xs1_deg, target_slice, [0 0], 'LineWidth', 2, 'Color', [0.8, 0.8, 0], 'LineStyle', '--');
        end
        
        % Shade the safe region
        safe_region = safe_slice <= 0;
        [rows, cols] = find(safe_region);
        if ~isempty(rows)
            h_safe_region = scatter(xs2_deg(sub2ind(size(xs2_deg), rows, cols)), ...
                    xs1_deg(sub2ind(size(xs1_deg), rows, cols)), ...
                    10, 'k', 'filled', 'MarkerFaceAlpha', 0.2);
        else
            % Create an empty scatter plot if no safe region exists
            h_safe_region = scatter([], [], 10, 'k', 'filled', 'MarkerFaceAlpha', 0.2);
        end
        
        % Add labels and legend
        xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
        ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
        title(sprintf('Safe Set Slice at δ = %.1f°', delta_values(slice_idx)*180/pi), 'FontSize', 14);
        
        % Set up legend with or without target
        if ~isempty(h_target)
            legend([h_brs, h_frs, h_safe, h_target], 'BRS', 'FRS', 'Safe Set', 'Target Set', 'Location', 'best');
        else
            legend([h_brs, h_frs, h_safe], 'BRS', 'FRS', 'Safe Set', 'Location', 'best');
        end
        
        % Set consistent axis limits
        x_limits = [g.min(2) * 180/pi, g.max(2) * 180/pi];  % Sideslip angle (beta)
        y_limits = [g.min(1) * 180/pi, g.max(1) * 180/pi];  % Yaw rate (gamma)
        xlim(ax_2d, x_limits);
        ylim(ax_2d, y_limits);
        
        grid(ax_2d, 'on');
    end

%% Nested function to initialize 3D visualization
    function [h_brs_3d, h_frs_3d, h_safe_3d, h_slice_plane, h_target_3d] = initialize_3d_view()
        % Create isosurfaces for each set
        isovalue = 0;  % The level set boundary
        
        % Plot the isosurfaces
        hold(ax_3d, 'on');
        
        % Create empty handles for surfaces
        h_brs_3d = [];
        h_frs_3d = [];
        h_safe_3d = [];
        h_target_3d = [];
        
        % BRS isosurface
        [brs_faces, brs_verts] = isosurface(beta_grid, gamma_grid, delta_grid, brs, isovalue);
        if ~isempty(brs_faces)
            h_brs_3d = patch(ax_3d, 'Faces', brs_faces, 'Vertices', brs_verts, ...
                     'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.2);
            % Use isonormals for better shading
            isonormals(beta_grid, gamma_grid, delta_grid, brs, h_brs_3d);
        end
        
        % FRS isosurface
        [frs_faces, frs_verts] = isosurface(beta_grid, gamma_grid, delta_grid, frs, isovalue);
        if ~isempty(frs_faces)
            h_frs_3d = patch(ax_3d, 'Faces', frs_faces, 'Vertices', frs_verts, ...
                     'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.2);
            % Use isonormals for better shading
            isonormals(beta_grid, gamma_grid, delta_grid, frs, h_frs_3d);
        end
        
        % Safe set isosurface (intersection)
        [safe_faces, safe_verts] = isosurface(beta_grid, gamma_grid, delta_grid, safe_set, isovalue);
        if ~isempty(safe_faces)
            h_safe_3d = patch(ax_3d, 'Faces', safe_faces, 'Vertices', safe_verts, ...
                     'FaceColor', 'green', 'EdgeColor', 'none', 'FaceAlpha', 0.4);
            % Use isonormals for better shading
            isonormals(beta_grid, gamma_grid, delta_grid, safe_set, h_safe_3d);
        end
        
        % Target set isosurface (if available)
        if ~isempty(target_set)
            [target_faces, target_verts] = isosurface(beta_grid, gamma_grid, delta_grid, target_set, isovalue);
            if ~isempty(target_faces)
                h_target_3d = patch(ax_3d, 'Faces', target_faces, 'Vertices', target_verts, ...
                         'FaceColor', [0.8, 0.8, 0], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
                % Use isonormals for better shading
                isonormals(beta_grid, gamma_grid, delta_grid, target_set, h_target_3d);
            end
        end
        
        % Create a slice plane to show where the current slice is located
        delta_val = delta_values(current_slice_idx) * 180/pi;
        
        % Create a plane at the current delta value
        [X, Y] = meshgrid(linspace(g.min(2), g.max(2), 20) * 180/pi, ...
                          linspace(g.min(1), g.max(1), 20) * 180/pi);
        Z = ones(size(X)) * delta_val;
        
        % Create a semi-transparent plane
        h_slice_plane = surf(ax_3d, X, Y, Z, 'FaceColor', [0.8 0.8 0.8], ...
                            'FaceAlpha', 0.6, 'EdgeColor', [0.5 0.5 0.5]);
        
        % Add labels
        xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
        ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
        zlabel('Steering Angle (degrees)', 'FontSize', 12);
        title('3D Safe Set Visualization', 'FontSize', 14);
        
        % Add legend if isosurfaces exist
        legend_handles = [];
        legend_labels = {};
        
        if ~isempty(h_brs_3d)
            legend_handles = [legend_handles, h_brs_3d];
            legend_labels{end+1} = 'BRS';
        end
        
        if ~isempty(h_frs_3d)
            legend_handles = [legend_handles, h_frs_3d];
            legend_labels{end+1} = 'FRS';
        end
        
        if ~isempty(h_safe_3d)
            legend_handles = [legend_handles, h_safe_3d];
            legend_labels{end+1} = 'Safe Set';
        end
        
        if ~isempty(h_target_3d)
            legend_handles = [legend_handles, h_target_3d];
            legend_labels{end+1} = 'Target Set';
        end
        
        if ~isempty(legend_handles)
            legend(ax_3d, legend_handles, legend_labels, 'Location', 'northeast');
        end
        
        % Set axis limits
        xlim(ax_3d, [g.min(2) * 180/pi, g.max(2) * 180/pi]);  % Sideslip angle (beta)
        ylim(ax_3d, [g.min(1) * 180/pi, g.max(1) * 180/pi]);  % Yaw rate (gamma)
        zlim(ax_3d, [g.min(3) * 180/pi, g.max(3) * 180/pi]);  % Steering angle (delta)
        
        % Set up lighting and view angle
        lighting gouraud;
        camlight('headlight');
        material dull;
        view(ax_3d, [-30, 30]);
        grid(ax_3d, 'on');
        
        % Enable 3D rotation
        rotate3d(ax_3d, 'on');
    end

%% Toggle visibility callback function
    function toggleVisibility(src, handle, set_name)
        if isempty(handle) || ~isvalid(handle)
            return; % Skip if handle is empty or invalid
        end
        
        % Get button state
        is_visible = get(src, 'Value');
        
        % Set visibility
        if is_visible
            set(handle, 'Visible', 'on');
        else
            set(handle, 'Visible', 'off');
        end
    end

%% Callback function for slider
    function update_slice(src, ~)
        % Get data from figure
        data = get(fig, 'UserData');
        
        % Get new slice index from slider
        slice_idx = round(get(src, 'Value'));
        
        % Update slice plane position
        delta_val = data.delta_values(slice_idx) * 180/pi;
        
        % Update text display
        set(data.text_display, 'String', sprintf('δ = %.1f°', delta_val));
        
        % Update 2D slice visualization
        axes(data.ax_2d);
        
        % Extract new 2D slices
        brs_slice = squeeze(data.brs(:,:,slice_idx));
        frs_slice = squeeze(data.frs(:,:,slice_idx));
        safe_slice = squeeze(data.safe_set(:,:,slice_idx));
        
        % Get target slice (use center slice for target)
        if ~isempty(data.target_set)
            target_center_idx = ceil(size(data.target_set, 3) / 2);
            target_slice = squeeze(data.target_set(:,:,target_center_idx));
        end
        
        % Get 2D grid coordinates
        xs1_deg = data.g.xs{1}(:,:,1) * 180/pi;
        xs2_deg = data.g.xs{2}(:,:,1) * 180/pi;
        
        % Update contours
        if isvalid(data.h_brs)
            delete(data.h_brs);
        end
        [~, data.h_brs] = contour(xs2_deg, xs1_deg, brs_slice, [0 0], 'LineWidth', 2, 'Color', 'b');
        
        if isvalid(data.h_frs)
            delete(data.h_frs);
        end
        [~, data.h_frs] = contour(xs2_deg, xs1_deg, frs_slice, [0 0], 'LineWidth', 2, 'Color', 'r');
        
        if isvalid(data.h_safe)
            delete(data.h_safe);
        end
        [~, data.h_safe] = contour(xs2_deg, xs1_deg, safe_slice, [0 0], 'LineWidth', 3, 'Color', 'k');
        
        % Update target if available
        if ~isempty(data.target_set) && isfield(data, 'h_target_2d') && ~isempty(data.h_target_2d)
            if isvalid(data.h_target_2d)
                delete(data.h_target_2d);
            end
            [~, data.h_target_2d] = contour(xs2_deg, xs1_deg, target_slice, [0 0], 'LineWidth', 2, 'Color', [0.8, 0.8, 0], 'LineStyle', '--');
        end
        
        % Update safe region shading
        if isvalid(data.h_safe_region)
            delete(data.h_safe_region);
        end
        
        % Shade the safe region
        safe_region = safe_slice <= 0;
        [rows, cols] = find(safe_region);
        if ~isempty(rows)
            data.h_safe_region = scatter(xs2_deg(sub2ind(size(xs2_deg), rows, cols)), ...
                    xs1_deg(sub2ind(size(xs1_deg), rows, cols)), ...
                    10, 'k', 'filled', 'MarkerFaceAlpha', 0.2);
        end
        
        % Update title
        title(data.ax_2d, sprintf('Safe Set Slice at δ = %.1f°', delta_val), 'FontSize', 14);
        
        % FIX: Recreate the legend with proper labels
        if ~isempty(data.target_set) && isfield(data, 'h_target_2d') && ~isempty(data.h_target_2d)
            legend([data.h_brs, data.h_frs, data.h_safe, data.h_target_2d], 'BRS', 'FRS', 'Safe Set', 'Target Set', 'Location', 'best');
        else
            legend([data.h_brs, data.h_frs, data.h_safe], 'BRS', 'FRS', 'Safe Set', 'Location', 'best');
        end
        
        % Update slice plane in 3D visualization
        if isvalid(data.h_slice_plane)
            % Update Z coordinate of the plane
            Z_coords = get(data.h_slice_plane, 'ZData');
            Z_coords(:) = delta_val;
            set(data.h_slice_plane, 'ZData', Z_coords);
        end
        
        % Store updated data back to figure
        set(fig, 'UserData', data);
    end
end