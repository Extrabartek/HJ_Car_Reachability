function visualizeCarTrajectory(traj, traj_tau, g, data_brs, data0, vx, varargin)
% VISUALIZECARTRAJECTORY Creates dual-view visualization of car trajectory
%
% This function creates a synchronized visualization with two views:
% 1. 3D state-space view (right): Shows trajectory in state space with BRS and target set
% 2. Top-down car-centric view (left): Shows car fixed at center with the world moving underneath
%
% Inputs:
%   traj        - State trajectory [gamma; beta; delta] over time
%                 gamma: yaw rate (rad/s)
%                 beta: side slip angle (rad)
%                 delta: steering angle (rad)
%   traj_tau    - Time points corresponding to trajectory
%   g           - Grid structure for state space
%   data_brs    - Backward reachable set data (value function from BRS computation)
%   data0       - Target set data (value function)
%   vx          - Constant longitudinal velocity (m/s)
%   varargin    - Optional parameter-value pairs:
%                 'x0'           - Initial x position (default: 0)
%                 'y0'           - Initial y position (default: 0)
%                 'psi0'         - Initial heading angle (default: 0)
%                 'saveVideo'    - Whether to save as video (default: false)
%                 'videoFile'    - Video filename (default: 'car_trajectory.mp4')
%                 'videoQuality' - Video quality (default: 95)
%                 'frameRate'    - Video frame rate (default: 30)
%                 'playSpeed'    - Playback speed multiplier (default: 1)
%                 'carLength'    - Car length in meters (default: 4.5)
%                 'carWidth'     - Car width in meters (default: 2.0)
%                 'wheelBase'    - Distance between axles (default: 2.7)
%                 'gridSize'     - Size of the grid in the car view (default: 20)
%                 'isoValue'     - Value for isosurface (default: 0)
%                 'brsOpacity'   - Opacity for BRS isosurface (default: 0.3)
%                 'targetOpacity'- Opacity for target isosurface (default: 0.5)

%% Parse inputs
p = inputParser;
p.addRequired('traj', @isnumeric);
p.addRequired('traj_tau', @isnumeric);
p.addRequired('g', @isstruct);
p.addRequired('data_brs', @isnumeric);
p.addRequired('data0', @isnumeric);
p.addRequired('vx', @isnumeric);
p.addParameter('x0', 0, @isnumeric);
p.addParameter('y0', 0, @isnumeric);
p.addParameter('psi0', 0, @isnumeric);
p.addParameter('saveVideo', false, @islogical);
p.addParameter('videoFile', 'car_trajectory.mp4', @ischar);
p.addParameter('videoQuality', 95, @(x) isnumeric(x) && x > 0 && x <= 100);
p.addParameter('frameRate', 30, @(x) isnumeric(x) && x > 0);
p.addParameter('playSpeed', 1, @(x) isnumeric(x) && x > 0);
p.addParameter('carLength', 4.5, @isnumeric);
p.addParameter('carWidth', 2.0, @isnumeric);
p.addParameter('wheelBase', 2.7, @isnumeric);
p.addParameter('gridSize', 20, @isnumeric);
p.addParameter('isoValue', 0, @isnumeric);
p.addParameter('brsOpacity', 0.3, @(x) isnumeric(x) && x >= 0 && x <= 1);
p.addParameter('targetOpacity', 0.5, @(x) isnumeric(x) && x >= 0 && x <= 1);

p.parse(traj, traj_tau, g, data_brs, data0, vx, varargin{:});
opts = p.Results;

% Check if we have a 3D BRS (needed for 3D visualization)
is_3d_brs = ndims(data_brs) == 3;
if ~is_3d_brs
    warning('BRS data is not 3D. The right panel will show a 2D view instead of 3D.');
end

%% Setup video recording if needed
try_video = opts.saveVideo;
if try_video
    try
        fprintf('Setting up video recording...\n');
        % Try with Motion JPEG first (more widely supported)
        v = VideoWriter(opts.videoFile, 'Motion JPEG AVI');
        v.Quality = opts.videoQuality;
        v.FrameRate = opts.frameRate;
        open(v);
        fprintf('Video recording setup successful using Motion JPEG AVI.\n');
    catch motion_jpeg_error
        try
            % If Motion JPEG fails, try uncompressed AVI
            fprintf('Motion JPEG failed, trying uncompressed AVI...\n');
            v = VideoWriter(opts.videoFile, 'Uncompressed AVI');
            v.FrameRate = opts.frameRate;
            open(v);
            fprintf('Video recording setup successful using Uncompressed AVI.\n');
        catch uncompressed_error
            % If both fail, disable video recording
            warning('Failed to initialize video recording. Continuing without recording.');
            fprintf('Error details: %s\n', uncompressed_error.message);
            try_video = false;
        end
    end
end

try
    %% Transform trajectory to physical coordinates
    fprintf('Transforming trajectory to physical coordinates...\n');
    pos_traj = transformTrajectory(traj, traj_tau, vx, ...
        'x0', opts.x0, 'y0', opts.y0, 'psi0', opts.psi0, 'visualize', false);

    % Extract trajectory components
    x = pos_traj.x;
    y = pos_traj.y;
    psi = pos_traj.psi;
    delta = pos_traj.delta;
    beta = pos_traj.beta;
    gamma = pos_traj.gamma;

    % Define parameter related to car dimensions
    wheel_width = opts.carWidth * 0.3;
    wheel_length = opts.carLength * 0.15;
    front_overhang = (opts.carLength - opts.wheelBase) * 0.4;
    rear_overhang = (opts.carLength - opts.wheelBase) * 0.6;

    %% Setup figure
    fig = figure('Name', 'Car Trajectory Visualization', ...
                 'Position', [50, 50, 1600, 800], ...
                 'Color', 'white');

    % Create layout with two main panels side by side
    left_panel = subplot(1, 2, 1);
    right_panel = subplot(1, 2, 2);

    %% Setup 3D state-space view (right panel)
    axes(right_panel);
    hold on;

    if is_3d_brs
        % Create 3D visualization with isosurfaces
        % Create grid matrices for isosurfaces - explicitly convert to degrees for visualization
        [beta_grid, gamma_grid, delta_grid] = meshgrid(...
            g.xs{2}(1,:,1) * 180/pi,...  % beta (sideslip angle)
            g.xs{1}(:,1,1) * 180/pi,...  % gamma (yaw rate)
            g.xs{3}(1,1,:) * 180/pi); % delta (steering angle)
        
        % Create BRS isosurface - first check if it will be empty
        [brs_faces, brs_verts] = isosurface(beta_grid, gamma_grid, delta_grid, data_brs, opts.isoValue);
        if ~isempty(brs_faces)
            h_brs = patch('Faces', brs_faces, 'Vertices', brs_verts, ...
                         'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', opts.brsOpacity);
        else
            h_brs = [];
            warning('BRS isosurface is empty. Will not display BRS boundary.');
        end
        
        % Create target set isosurface - first check if it will be empty
        [target_faces, target_verts] = isosurface(beta_grid, gamma_grid, delta_grid, data0, opts.isoValue);
        if ~isempty(target_faces)
            h_target = patch('Faces', target_faces, 'Vertices', target_verts, ...
                           'FaceColor', 'green', 'EdgeColor', 'none', 'FaceAlpha', opts.targetOpacity);
        else
            h_target = [];
            warning('Target isosurface is empty. Will not display target set.');
        end
        
        % Plot the 3D trajectory
        h_traj = plot3(traj(2,:) * 180/pi, traj(1,:) * 180/pi, traj(3,:) * 180/pi, ...
            'b-', 'LineWidth', 2);
        
        % Mark start and end points
        h_start = plot3(traj(2,1) * 180/pi, traj(1,1) * 180/pi, traj(3,1) * 180/pi, ...
            'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        h_end = plot3(traj(2,end) * 180/pi, traj(1,end) * 180/pi, traj(3,end) * 180/pi, ...
            'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        
        % Current position marker (will be updated during animation)
        h_current = plot3(traj(2,1) * 180/pi, traj(1,1) * 180/pi, traj(3,1) * 180/pi, ...
            'ro', 'MarkerSize', 12, 'LineWidth', 2);
        
        % Add labels and enhance 3D visualization
        xlabel('Sideslip Angle \beta (deg)', 'FontSize', 12);
        ylabel('Yaw Rate \gamma (deg/s)', 'FontSize', 12);
        zlabel('Steering Angle \delta (deg)', 'FontSize', 12);
        title('3D State Space View', 'FontSize', 14);
        
        % Set up lighting and view angle for better 3D visualization
        lighting gouraud;
        camlight('headlight');
        material dull;
        view([-30, 30]);
        
        % Add legend with valid handles only
        % First collect valid handles and their labels
        legend_handles = [];
        legend_labels = {};
        
        if ~isempty(h_brs) && ishandle(h_brs) && isvalid(h_brs)
            legend_handles = [legend_handles, h_brs];
            legend_labels{end+1} = 'BRS Boundary';
        end
        
        if ~isempty(h_target) && ishandle(h_target) && isvalid(h_target)
            legend_handles = [legend_handles, h_target];
            legend_labels{end+1} = 'Target Set';
        end
        
        if ~isempty(h_traj) && ishandle(h_traj)
            legend_handles = [legend_handles, h_traj];
            legend_labels{end+1} = 'Trajectory';
        end
        
        if ~isempty(h_start) && ishandle(h_start)
            legend_handles = [legend_handles, h_start];
            legend_labels{end+1} = 'Start';
        end
        
        if ~isempty(h_end) && ishandle(h_end)
            legend_handles = [legend_handles, h_end];
            legend_labels{end+1} = 'End';
        end
        
        if ~isempty(h_current) && ishandle(h_current)
            legend_handles = [legend_handles, h_current];
            legend_labels{end+1} = 'Current Position';
        end
        
        % Create legend only if we have valid handles
        if ~isempty(legend_handles)
            legend(legend_handles, legend_labels, 'Location', 'northeast', 'FontSize', 10);
        end
        
        % Enable rotation and set axis bounds
        axis tight;
        grid on;
        box on;
    else
        % Fallback to 2D visualization if BRS is only 2D
        % Plot BRS boundary
        [C_brs, h_brs] = contour(g.xs{2}, g.xs{1}, data_brs, [0 0], 'r-', 'LineWidth', 2);
        hold on;

        % Plot target set
        [C_target, h_target] = contour(g.xs{2}, g.xs{1}, data0, [0 0], 'g-', 'LineWidth', 2);

        % Plot full trajectory
        h_traj = plot(traj(2,:) * 180/pi, traj(1,:) * 180/pi, 'b-', 'LineWidth', 1.5);

        % Mark start and end points
        h_start = plot(traj(2,1) * 180/pi, traj(1,1) * 180/pi, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        h_end = plot(traj(2,end) * 180/pi, traj(1,end) * 180/pi, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

        % Current position marker (will be updated during animation)
        h_current = plot(traj(2,1) * 180/pi, traj(1,1) * 180/pi, 'ro', 'MarkerSize', 12, 'LineWidth', 2);

        % Add labels and legend with valid handles only
        grid on;
        xlabel('Sideslip Angle \beta (deg)', 'FontSize', 12);
        ylabel('Yaw Rate \gamma (deg/s)', 'FontSize', 12);
        title('State Space View (2D Projection)', 'FontSize', 14);
        
        % Collect valid handles and their labels
        legend_handles = [];
        legend_labels = {};
        
        if ~isempty(h_brs) && ishandle(h_brs)
            legend_handles = [legend_handles, h_brs];
            legend_labels{end+1} = 'BRS Boundary';
        end
        
        if ~isempty(h_target) && ishandle(h_target)
            legend_handles = [legend_handles, h_target];
            legend_labels{end+1} = 'Target Set';
        end
        
        if ~isempty(h_traj) && ishandle(h_traj)
            legend_handles = [legend_handles, h_traj];
            legend_labels{end+1} = 'Trajectory';
        end
        
        if ~isempty(h_start) && ishandle(h_start)
            legend_handles = [legend_handles, h_start];
            legend_labels{end+1} = 'Start';
        end
        
        if ~isempty(h_end) && ishandle(h_end)
            legend_handles = [legend_handles, h_end];
            legend_labels{end+1} = 'End';
        end
        
        if ~isempty(h_current) && ishandle(h_current)
            legend_handles = [legend_handles, h_current];
            legend_labels{end+1} = 'Current Position';
        end
        
        % Create legend only if we have valid handles
        if ~isempty(legend_handles)
            legend(legend_handles, legend_labels, 'Location', 'best', 'FontSize', 10);
        end

        % Set reasonable axis limits
        axis tight;
        ax_limits = axis();
        axis_margin = 0.1 * max(ax_limits(2)-ax_limits(1), ax_limits(4)-ax_limits(3));
        axis([ax_limits(1)-axis_margin, ax_limits(2)+axis_margin, ...
              ax_limits(3)-axis_margin, ax_limits(4)+axis_margin]);
    end

    %% Setup top-down car-centric view (left panel)
    axes(left_panel);

    % Define grid size
    grid_size = opts.gridSize;
    grid_spacing = 2;  % meters between grid lines
    visible_grid_size = grid_size * grid_spacing;

    % Create initial grid (this will be updated during animation)
    [grid_x, grid_y] = meshgrid(-visible_grid_size/2:grid_spacing:visible_grid_size/2, ...
                               -visible_grid_size/2:grid_spacing:visible_grid_size/2);
                           
    % Convert the meshgrid outputs to vectors for plotting
    grid_x_vec = grid_x(:);
    grid_y_vec = grid_y(:);
    
    % Create the grid with vectors instead of matrices
    h_grid = plot(grid_x_vec, grid_y_vec, 'k.', 'MarkerSize', 3);
    hold on;
    
    % Add perpendicular grid lines
    h_grid_perp = plot(grid_y_vec, grid_x_vec, 'k.', 'MarkerSize', 3);

    %% NEW: Calculate the initial sideslip offset to align velocity vector with car frame
    % Get initial sideslip angle
    initial_beta = beta(1);
    
    % The car should be rotated by -initial_beta to align velocity vector with car's x-axis
    car_rotation = -initial_beta;

    % Plot the full path
    h_full_path = plot(x, y, 'b-', 'LineWidth', 2);
    
    % Mark start and end positions
    plot(x(1), y(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    plot(x(end), y(end), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    
    % Create car shape - pre-rotated to match initial velocity direction
    car_color = [0.3, 0.5, 0.8];  % Light blue
    
    % Create original car outline
    orig_car_outline = [
        -rear_overhang, opts.carWidth/2;   % Rear left
        opts.wheelBase + front_overhang, opts.carWidth/2;   % Front left
        opts.wheelBase + front_overhang, -opts.carWidth/2;  % Front right
        -rear_overhang, -opts.carWidth/2;  % Rear right
        -rear_overhang, opts.carWidth/2    % Back to rear left (close the shape)
    ];
    
    % Rotate car outline to match initial velocity direction
    car_outline = zeros(size(orig_car_outline));
    for i = 1:size(orig_car_outline, 1)
        % Rotate each point
        car_outline(i,1) = orig_car_outline(i,1) * cos(car_rotation) - orig_car_outline(i,2) * sin(car_rotation);
        car_outline(i,2) = orig_car_outline(i,1) * sin(car_rotation) + orig_car_outline(i,2) * cos(car_rotation);
    end
    
    % Create car shape
    h_car = patch('XData', car_outline(:,1), 'YData', car_outline(:,2), ...
                  'FaceColor', car_color, 'EdgeColor', 'k', 'LineWidth', 1.5);

    % Create wheels (rectangles) - original positions
    orig_front_left_wheel = [
        opts.wheelBase, wheel_width/2;
        opts.wheelBase + wheel_length, wheel_width/2;
        opts.wheelBase + wheel_length, -wheel_width/2;
        opts.wheelBase, -wheel_width/2;
        opts.wheelBase, wheel_width/2
    ];

    orig_front_right_wheel = orig_front_left_wheel;
    orig_front_right_wheel(:,2) = -orig_front_left_wheel(:,2) - wheel_width;

    orig_rear_left_wheel = orig_front_left_wheel;
    orig_rear_left_wheel(:,1) = orig_rear_left_wheel(:,1) - opts.wheelBase;

    orig_rear_right_wheel = orig_front_right_wheel;
    orig_rear_right_wheel(:,1) = orig_rear_right_wheel(:,1) - opts.wheelBase;
    
    % Rotate wheels to match initial velocity direction
    front_left_wheel = zeros(size(orig_front_left_wheel));
    front_right_wheel = zeros(size(orig_front_right_wheel));
    rear_left_wheel = zeros(size(orig_rear_left_wheel));
    rear_right_wheel = zeros(size(orig_rear_right_wheel));
    
    % Calculate wheel centers before rotation
    fl_center_x = mean(orig_front_left_wheel(:,1));
    fl_center_y = mean(orig_front_left_wheel(:,2) + opts.carWidth/2 - wheel_width/2);
    fr_center_x = mean(orig_front_right_wheel(:,1));
    fr_center_y = mean(orig_front_right_wheel(:,2) - opts.carWidth/2 + wheel_width/2);
    rl_center_x = mean(orig_rear_left_wheel(:,1));
    rl_center_y = mean(orig_rear_left_wheel(:,2) + opts.carWidth/2 - wheel_width/2);
    rr_center_x = mean(orig_rear_right_wheel(:,1));
    rr_center_y = mean(orig_rear_right_wheel(:,2) - opts.carWidth/2 + wheel_width/2);
    
    % Rotate wheel centers
    fl_rotated_center_x = fl_center_x * cos(car_rotation) - fl_center_y * sin(car_rotation);
    fl_rotated_center_y = fl_center_x * sin(car_rotation) + fl_center_y * cos(car_rotation);
    fr_rotated_center_x = fr_center_x * cos(car_rotation) - fr_center_y * sin(car_rotation);
    fr_rotated_center_y = fr_center_x * sin(car_rotation) + fr_center_y * cos(car_rotation);
    rl_rotated_center_x = rl_center_x * cos(car_rotation) - rl_center_y * sin(car_rotation);
    rl_rotated_center_y = rl_center_x * sin(car_rotation) + rl_center_y * cos(car_rotation);
    rr_rotated_center_x = rr_center_x * cos(car_rotation) - rr_center_y * sin(car_rotation);
    rr_rotated_center_y = rr_center_x * sin(car_rotation) + rr_center_y * cos(car_rotation);
    
    % Rotate each wheel shape
    for i = 1:size(orig_front_left_wheel, 1)
        % Front left wheel
        x_offset = orig_front_left_wheel(i,1) - fl_center_x;
        y_offset = (orig_front_left_wheel(i,2) + opts.carWidth/2 - wheel_width/2) - fl_center_y;
        
        x_rotated = x_offset * cos(car_rotation) - y_offset * sin(car_rotation);
        y_rotated = x_offset * sin(car_rotation) + y_offset * cos(car_rotation);
        
        front_left_wheel(i,1) = fl_rotated_center_x + x_rotated;
        front_left_wheel(i,2) = fl_rotated_center_y + y_rotated;
        
        % Front right wheel
        x_offset = orig_front_right_wheel(i,1) - fr_center_x;
        y_offset = (orig_front_right_wheel(i,2) - opts.carWidth/2 + wheel_width/2) - fr_center_y;
        
        x_rotated = x_offset * cos(car_rotation) - y_offset * sin(car_rotation);
        y_rotated = x_offset * sin(car_rotation) + y_offset * cos(car_rotation);
        
        front_right_wheel(i,1) = fr_rotated_center_x + x_rotated;
        front_right_wheel(i,2) = fr_rotated_center_y + y_rotated;
        
        % Rear left wheel
        x_offset = orig_rear_left_wheel(i,1) - rl_center_x;
        y_offset = (orig_rear_left_wheel(i,2) + opts.carWidth/2 - wheel_width/2) - rl_center_y;
        
        x_rotated = x_offset * cos(car_rotation) - y_offset * sin(car_rotation);
        y_rotated = x_offset * sin(car_rotation) + y_offset * cos(car_rotation);
        
        rear_left_wheel(i,1) = rl_rotated_center_x + x_rotated;
        rear_left_wheel(i,2) = rl_rotated_center_y + y_rotated;
        
        % Rear right wheel
        x_offset = orig_rear_right_wheel(i,1) - rr_center_x;
        y_offset = (orig_rear_right_wheel(i,2) - opts.carWidth/2 + wheel_width/2) - rr_center_y;
        
        x_rotated = x_offset * cos(car_rotation) - y_offset * sin(car_rotation);
        y_rotated = x_offset * sin(car_rotation) + y_offset * cos(car_rotation);
        
        rear_right_wheel(i,1) = rr_rotated_center_x + x_rotated;
        rear_right_wheel(i,2) = rr_rotated_center_y + y_rotated;
    end

    % Create wheel patches - no need to add offsets since already included in the rotated wheel coordinates
    h_wheel_fl = patch('XData', front_left_wheel(:,1), 'YData', front_left_wheel(:,2), ...
                      'FaceColor', 'k', 'EdgeColor', 'k');
    h_wheel_fr = patch('XData', front_right_wheel(:,1), 'YData', front_right_wheel(:,2), ...
                      'FaceColor', 'k', 'EdgeColor', 'k');
    h_wheel_rl = patch('XData', rear_left_wheel(:,1), 'YData', rear_left_wheel(:,2), ...
                      'FaceColor', 'k', 'EdgeColor', 'k');
    h_wheel_rr = patch('XData', rear_right_wheel(:,1), 'YData', rear_right_wheel(:,2), ...
                      'FaceColor', 'k', 'EdgeColor', 'k');

    % Add direction indicator - now aligned with car's frame but considering initial rotation
    arrow_length = opts.carLength * 0.6;
    initial_arrow_angle = car_rotation + delta(1); % Initial steering angle plus car rotation
    h_direction = quiver(0, 0, arrow_length * cos(initial_arrow_angle), arrow_length * sin(initial_arrow_angle), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);


    % Create velocity vector arrow (initially at origin)
    h_velocity_vector = quiver(0, 0, 0, 0, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);

    % Set up axes properties
    axis equal;
    grid on;
    xlabel('X Position (m)', 'FontSize', 12);
    ylabel('Y Position (m)', 'FontSize', 12);
    title('Car-Centric View', 'FontSize', 14);

    visible_grid_size_with_margin = visible_grid_size * 0.6;
    xlim([min(x)*1.1, max(x)*1.1]);
    ylim([min(y)*1.1, max(y)*1.1]);

    %% Create progress bar, time display, and play/pause button
    % Slider for time control
    progress_bar = uicontrol('Style', 'slider', ...
                            'Min', 1, 'Max', length(traj_tau), ...
                            'Value', 1, ...
                            'SliderStep', [1/length(traj_tau), 10/length(traj_tau)], ...
                            'Position', [200, 20, 1000, 20]);

    % Time display
    time_display = uicontrol('Style', 'text', ...
                             'String', sprintf('Time: %.2f s', traj_tau(1)), ...
                             'Position', [800, 45, 200, 20], ...
                             'BackgroundColor', 'white', ...
                             'FontSize', 12);

    % Play/Pause button (only when not saving video)
    if ~try_video
        play_button = uicontrol('Style', 'togglebutton', ...
                               'String', 'Play', ...
                               'Position', [50, 20, 100, 30], ...
                               'Value', 0);
    end

    % Add title to the figure
    sgtitle(sprintf('Car Trajectory Visualization (v_x = %.1f m/s)', vx), ...
            'FontSize', 16, 'FontWeight', 'bold');
    
    % Store variables needed by nested functions in a struct for reference
    userData = struct(...
        'traj_tau', traj_tau, ...
        'traj', traj, ...
        'gamma', gamma, ...
        'beta', beta, ...
        'delta', delta, ...
        'psi', psi, ...
        'x', x, ...
        'y', y, ...
        'h_current', h_current, ...
        'visible_grid_size', visible_grid_size, ...
        'grid_spacing', grid_spacing, ...
        'h_grid', h_grid, ...
        'h_car', h_car, ...
        'h_grid_perp', h_grid_perp, ...
        'h_wheel_fl', h_wheel_fl, ...
        'h_wheel_fr', h_wheel_fr, ...
        'h_wheel_rl', h_wheel_rl, ...
        'h_wheel_rr', h_wheel_rr, ...
        'fl_center_x', fl_rotated_center_x, ...
        'fl_center_y', fl_rotated_center_y, ...
        'fr_center_x', fr_rotated_center_x, ...
        'fr_center_y', fr_rotated_center_y, ...
        'car_rotation', car_rotation, ...
        'front_left_wheel', front_left_wheel, ...
        'front_right_wheel', front_right_wheel, ...
        'rear_left_wheel', rear_left_wheel, ...
        'rear_right_wheel', rear_right_wheel, ...
        'opts', opts, ...
        'wheel_width', wheel_width, ...
        'h_direction', h_direction, ...
        'arrow_length', arrow_length, ...
        'h_path', h_full_path, ...
        'visible_grid_size_with_margin', visible_grid_size_with_margin, ...
        'is_3d_brs', is_3d_brs, ...
        'time_display', time_display, ...
        'left_panel', left_panel, ...
        'car_outline', car_outline, ...
        'h_velocity_vector', h_velocity_vector ...
        );
    
    % Store the userData in the figure for access by callbacks
    set(fig, 'UserData', userData);

    % Setup callback for slider
    set(progress_bar, 'Callback', @updateVisualizationCallback);
    
    % Setup callback for play button
    if ~try_video
        is_playing = false;
        play_timer = timer('Period', 0.05, 'ExecutionMode', 'fixedRate');
        
        % Timer callback function
        play_timer.TimerFcn = @updateFromTimerCallback;
        
        % Play button callback
        set(play_button, 'Callback', @togglePlaybackCallback);
        
        % Store timer in figure UserData
        userData.play_timer = play_timer;
        userData.progress_bar = progress_bar;
        userData.is_playing = is_playing;
        set(fig, 'UserData', userData);
    end
    
    %% Animation or interactive mode
    if try_video
        % Animation loop for video recording
        fprintf('Starting animation for video recording...\n');
        
        % Calculate number of frames and frame times
        total_time = traj_tau(end) - traj_tau(1);
        
        % For video: create a specific number of evenly-spaced frames
        num_frames = ceil(total_time * opts.frameRate / opts.playSpeed);
        frame_times = linspace(traj_tau(1), traj_tau(end), num_frames);
        
        % Animation loop
        for frame = 1:num_frames
            % Get the current time for this frame
            current_time = frame_times(frame);
            
            % Find the index in the original trajectory closest to current time
            [~, idx] = min(abs(traj_tau - current_time));
            
            % Update visualization
            updateVisualization(idx, userData);
            
            % Capture frame for video
            frame_data = getframe(fig);
            writeVideo(v, frame_data);
            
            % Show progress
            if mod(frame, 10) == 0
                fprintf('Recording: %.1f%% complete\n', 100 * frame / num_frames);
            end
        end
        
        % Close video file
        close(v);
        fprintf('Video saved successfully to: %s\n', opts.videoFile);
    else
        % Interactive mode - show the initial frame
        updateVisualization(1, userData);
        fprintf('Interactive mode ready. Use the slider to navigate or click Play to animate.\n');
    end

    % Setup cleanup when figure is closed
    set(fig, 'CloseRequestFcn', @onClose);
    
    fprintf('Visualization complete.\n');
    
catch err
    % Handle any errors
    fprintf('Visualization failed: %s\n', err.message);
    
    % If we were trying to record video and it failed, try without video
    if try_video
        try_video = false;
        fprintf('Attempting visualization without video recording...\n');
        visualizeCarTrajectory(traj, traj_tau, g, data_brs, data0, vx, ...
            'x0', opts.x0, 'y0', opts.y0, 'psi0', opts.psi0, ...
            'saveVideo', false, ...
            'carLength', opts.carLength, ...
            'carWidth', opts.carWidth, ...
            'wheelBase', opts.wheelBase, ...
            'gridSize', opts.gridSize);
    end
end

end

% Helper function to rotate points
function rotated_points = rotatePoints(points, angle)
    % Create rotation matrix
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    
    % Rotate points
    rotated_points = (R * points')';
end

% Helper function to translate points
function translated_points = translatePoints(points, translation)
    % Add translation vector to each point
    translated_points = points + repmat(translation, size(points, 1), 1);
end

% Callback function for the slider - will have access to figure's UserData
function updateVisualizationCallback(src, ~)
    % Get the userData from the figure
    fig = get(src, 'Parent');
    userData = get(fig, 'UserData');
    
    % Get the current index from the slider
    idx = round(get(src, 'Value'));
    
    % Call the updateVisualization function with the userData
    updateVisualization(idx, userData);
end

% Callback function for the timer - will have access to figure's UserData
function updateFromTimerCallback(~, ~)
    % Find the figure with our visualization
    fig = findobj('Type', 'figure', 'Name', 'Car Trajectory Visualization');
    if isempty(fig)
        return;
    end
    
    % Get the userData
    userData = get(fig, 'UserData');
    
    % Get current index 
    current_idx = round(get(userData.progress_bar, 'Value'));
    next_idx = current_idx + 1;
    
    % Check if we've reached the end
    if next_idx > length(userData.traj_tau)
        next_idx = 1;  % Loop back to beginning
        % Or stop: 
        % set(play_button, 'Value', 0, 'String', 'Play');
        % userData.is_playing = false;
        % stop(userData.play_timer);
        % set(fig, 'UserData', userData);
        % return;
    end
    
    % Update slider position
    set(userData.progress_bar, 'Value', next_idx);
    
    % Update visualization
    updateVisualization(next_idx, userData);
end

% Callback function for play/pause button
function togglePlaybackCallback(src, ~)
    % Get the figure and its userData
    fig = get(src, 'Parent');
    userData = get(fig, 'UserData');
    
    % Update playing state
    userData.is_playing = get(src, 'Value');
    set(fig, 'UserData', userData);
    
    if userData.is_playing
        % Start playback
        set(src, 'String', 'Pause');
        start(userData.play_timer);
    else
        % Pause playback
        set(src, 'String', 'Play');
        stop(userData.play_timer);
    end
end

% Function to handle figure closing
function onClose(src, ~)
    % Get userData from the figure
    userData = get(src, 'UserData');
    
    % Clean up timer if it exists
    if isfield(userData, 'play_timer') && isvalid(userData.play_timer)
        stop(userData.play_timer);
        delete(userData.play_timer);
    end
    
    % Close figure
    delete(src);
end

function updateVisualization(idx, userData)
    % Get current state
    current_time = userData.traj_tau(idx);
    current_gamma = userData.gamma(idx);
    current_beta = userData.beta(idx);
    current_delta = userData.delta(idx);
    current_psi = userData.psi(idx);
    current_x = userData.x(idx);
    current_y = userData.y(idx);
    
    % Update time display
    set(userData.time_display, 'String', sprintf('Time: %.2f s', current_time));
    
    % Update state-space view
    if userData.is_3d_brs
        % Update 3D state-space marker
        if ishandle(userData.h_current)
            set(userData.h_current, 'XData', current_beta * 180/pi, ...
                         'YData', current_gamma * 180/pi, ...
                         'ZData', current_delta * 180/pi);
        end
    else
        % Update 2D state-space marker
        if ishandle(userData.h_current)
            set(userData.h_current, 'XData', current_beta * 180/pi, ...
                         'YData', current_gamma * 180/pi);
        end
    end
    
    % ===== FIX: PROPERLY ROTATE AND POSITION THE CAR AND WHEELS =====
    
    % 1. Get car dimensions from userData
    car_length = userData.opts.carLength;
    car_width = userData.opts.carWidth;
    wheel_base = userData.opts.wheelBase;
    front_overhang = (car_length - wheel_base) * 0.4;
    rear_overhang = (car_length - wheel_base) * 0.6;
    wheel_width = car_width * 0.3;
    wheel_length = car_length * 0.15;
    
    % 2. Create car shape in local coordinates (centered at origin)
    car_outline = [
        -rear_overhang, car_width/2;   % Rear left
        wheel_base + front_overhang, car_width/2;   % Front left
        wheel_base + front_overhang, -car_width/2;  % Front right
        -rear_overhang, -car_width/2;  % Rear right
        -rear_overhang, car_width/2    % Back to rear left (close the shape)
    ];
    
    % 3. Create wheel shapes in local coordinates
    % Rear left wheel
    rl_wheel_x = 0; % At the rear axle
    rl_wheel_y = car_width/2 - wheel_width/2;
    rear_left_wheel = [
        rl_wheel_x, rl_wheel_y;
        rl_wheel_x + wheel_length, rl_wheel_y;
        rl_wheel_x + wheel_length, rl_wheel_y - wheel_width;
        rl_wheel_x, rl_wheel_y - wheel_width;
        rl_wheel_x, rl_wheel_y
    ];
    
    % Rear right wheel
    rr_wheel_x = 0; % At the rear axle
    rr_wheel_y = -car_width/2 + wheel_width/2;
    rear_right_wheel = [
        rr_wheel_x, rr_wheel_y;
        rr_wheel_x + wheel_length, rr_wheel_y;
        rr_wheel_x + wheel_length, rr_wheel_y - wheel_width;
        rr_wheel_x, rr_wheel_y - wheel_width;
        rr_wheel_x, rr_wheel_y
    ];
    
    % Front left wheel (will be rotated by steering angle later)
    fl_wheel_x = wheel_base; % At the front axle
    fl_wheel_y = car_width/2 - wheel_width/2;
    front_left_wheel = [
        fl_wheel_x, fl_wheel_y;
        fl_wheel_x + wheel_length, fl_wheel_y;
        fl_wheel_x + wheel_length, fl_wheel_y - wheel_width;
        fl_wheel_x, fl_wheel_y - wheel_width;
        fl_wheel_x, fl_wheel_y
    ];
    
    % Front right wheel (will be rotated by steering angle later)
    fr_wheel_x = wheel_base; % At the front axle
    fr_wheel_y = -car_width/2 + wheel_width/2;
    front_right_wheel = [
        fr_wheel_x, fr_wheel_y;
        fr_wheel_x + wheel_length, fr_wheel_y;
        fr_wheel_x + wheel_length, fr_wheel_y - wheel_width;
        fr_wheel_x, fr_wheel_y - wheel_width;
        fr_wheel_x, fr_wheel_y
    ];
    
    % 4. Apply steering angle to front wheels
    % Calculate the center of front left wheel
    fl_center_x = fl_wheel_x + wheel_length/2;
    fl_center_y = fl_wheel_y - wheel_width/2;
    
    % Calculate the center of front right wheel
    fr_center_x = fr_wheel_x + wheel_length/2;
    fr_center_y = fr_wheel_y - wheel_width/2;
    
    % Steer front wheels (rotate around their centers)
    % Front left wheel
    rotated_fl = zeros(size(front_left_wheel));
    for i = 1:size(front_left_wheel, 1)
        % Calculate offset from center
        x_offset = front_left_wheel(i,1) - fl_center_x;
        y_offset = front_left_wheel(i,2) - fl_center_y;
        
        % Rotate by steering angle
        x_rotated = x_offset * cos(-current_delta) - y_offset * sin(-current_delta);
        y_rotated = x_offset * sin(-current_delta) + y_offset * cos(-current_delta);
        
        % Move back to position relative to center
        rotated_fl(i,1) = fl_center_x + x_rotated;
        rotated_fl(i,2) = fl_center_y + y_rotated;
    end
    
    % Front right wheel
    rotated_fr = zeros(size(front_right_wheel));
    for i = 1:size(front_right_wheel, 1)
        % Calculate offset from center
        x_offset = front_right_wheel(i,1) - fr_center_x;
        y_offset = front_right_wheel(i,2) - fr_center_y;
        
        % Rotate by steering angle
        x_rotated = x_offset * cos(-current_delta) - y_offset * sin(-current_delta);
        y_rotated = x_offset * sin(-current_delta) + y_offset * cos(-current_delta);
        
        % Move back to position relative to center
        rotated_fr(i,1) = fr_center_x + x_rotated;
        rotated_fr(i,2) = fr_center_y + y_rotated;
    end
    
    % 5. Apply car heading (psi) rotation to everything
    % First rotate car outline
    rotated_car = zeros(size(car_outline));
    for i = 1:size(car_outline, 1)
        rotated_car(i,1) = car_outline(i,1) * cos(current_psi) - car_outline(i,2) * sin(current_psi);
        rotated_car(i,2) = car_outline(i,1) * sin(current_psi) + car_outline(i,2) * cos(current_psi);
    end
    
    % Rotate rear wheels
    rotated_rl = zeros(size(rear_left_wheel));
    for i = 1:size(rear_left_wheel, 1)
        rotated_rl(i,1) = rear_left_wheel(i,1) * cos(current_psi) - rear_left_wheel(i,2) * sin(current_psi);
        rotated_rl(i,2) = rear_left_wheel(i,1) * sin(current_psi) + rear_left_wheel(i,2) * cos(current_psi);
    end
    
    rotated_rr = zeros(size(rear_right_wheel));
    for i = 1:size(rear_right_wheel, 1)
        rotated_rr(i,1) = rear_right_wheel(i,1) * cos(current_psi) - rear_right_wheel(i,2) * sin(current_psi);
        rotated_rr(i,2) = rear_right_wheel(i,1) * sin(current_psi) + rear_right_wheel(i,2) * cos(current_psi);
    end
    
    % Apply heading rotation to already-steered front wheels
    rotated_fl_final = zeros(size(rotated_fl));
    for i = 1:size(rotated_fl, 1)
        rotated_fl_final(i,1) = rotated_fl(i,1) * cos(current_psi) - rotated_fl(i,2) * sin(current_psi);
        rotated_fl_final(i,2) = rotated_fl(i,1) * sin(current_psi) + rotated_fl(i,2) * cos(current_psi);
    end
    
    rotated_fr_final = zeros(size(rotated_fr));
    for i = 1:size(rotated_fr, 1)
        rotated_fr_final(i,1) = rotated_fr(i,1) * cos(current_psi) - rotated_fr(i,2) * sin(current_psi);
        rotated_fr_final(i,2) = rotated_fr(i,1) * sin(current_psi) + rotated_fr(i,2) * cos(current_psi);
    end
    
    % 6. Translate everything to current position
    translated_car = zeros(size(rotated_car));
    for i = 1:size(rotated_car, 1)
        translated_car(i,1) = rotated_car(i,1) + current_x;
        translated_car(i,2) = rotated_car(i,2) + current_y;
    end
    
    translated_rl = zeros(size(rotated_rl));
    for i = 1:size(rotated_rl, 1)
        translated_rl(i,1) = rotated_rl(i,1) + current_x;
        translated_rl(i,2) = rotated_rl(i,2) + current_y;
    end
    
    translated_rr = zeros(size(rotated_rr));
    for i = 1:size(rotated_rr, 1)
        translated_rr(i,1) = rotated_rr(i,1) + current_x;
        translated_rr(i,2) = rotated_rr(i,2) + current_y;
    end
    
    translated_fl = zeros(size(rotated_fl_final));
    for i = 1:size(rotated_fl_final, 1)
        translated_fl(i,1) = rotated_fl_final(i,1) + current_x;
        translated_fl(i,2) = rotated_fl_final(i,2) + current_y;
    end
    
    translated_fr = zeros(size(rotated_fr_final));
    for i = 1:size(rotated_fr_final, 1)
        translated_fr(i,1) = rotated_fr_final(i,1) + current_x;
        translated_fr(i,2) = rotated_fr_final(i,2) + current_y;
    end
    
    % 7. Update all graphical objects
    set(userData.h_car, 'XData', translated_car(:,1), 'YData', translated_car(:,2));
    set(userData.h_wheel_rl, 'XData', translated_rl(:,1), 'YData', translated_rl(:,2));
    set(userData.h_wheel_rr, 'XData', translated_rr(:,1), 'YData', translated_rr(:,2));
    set(userData.h_wheel_fl, 'XData', translated_fl(:,1), 'YData', translated_fl(:,2));
    set(userData.h_wheel_fr, 'XData', translated_fr(:,1), 'YData', translated_fr(:,2));
    
    % 8. Update direction indicator (steering direction arrow)
    arrow_length = userData.arrow_length;
    arrow_angle = current_psi - current_delta;
    set(userData.h_direction, 'XData', current_x, 'YData', current_y, ...
                   'UData', arrow_length * cos(arrow_angle), ...
                   'VData', arrow_length * sin(arrow_angle));

    % Calculate and update velocity vector
    % Get velocity components in body frame
    vx_body = userData.opts.vx;  % Constant longitudinal velocity
    vy_body = vx_body * tan(current_beta);  % Lateral velocity from sideslip angle
    
    % Rotate to global frame
    vx_global = vx_body * cos(current_psi) - vy_body * sin(current_psi);
    vy_global = vx_body * sin(current_psi) + vy_body * cos(current_psi);
    
    % Scale for visualization (adjust scale_factor as needed for good visibility)
    scale_factor = 0.2;
    vx_scaled = vx_global * scale_factor;
    vy_scaled = vy_global * scale_factor;
    
    % Update velocity vector
    set(userData.h_velocity_vector, 'XData', current_x, 'YData', current_y, ...
                      'UData', vx_scaled, 'VData', vy_scaled);
    
    % 9. Update any other elements as needed (like the grid points, path, etc.)
    % (The existing code for those elements can remain unchanged)
    
    % Update display
    drawnow;
end