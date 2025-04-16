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
%                 'deltaSlice'   - Steering angle slice for BRS (default: middle slice)
%
% Example:
%   visualizeCarTrajectory(traj, traj_tau, g, data_brs, data0, 30, 'saveVideo', true);

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
p.addParameter('deltaSlice', [], @(x) isempty(x) || isnumeric(x));

p.parse(traj, traj_tau, g, data_brs, data0, vx, varargin{:});
opts = p.Results;

% Determine delta slice if not specified
if isempty(opts.deltaSlice) && ndims(data_brs) > 2
    opts.deltaSlice = ceil(size(data_brs, 3)/2);
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

    % Extract a 2D slice of BRS at the specified steering angle if 3D
    if ndims(data_brs) == 3
        brs_2d_slice = squeeze(data_brs(:,:,opts.deltaSlice));
        target_2d_slice = squeeze(data0(:,:,opts.deltaSlice));
        delta_values = linspace(g.min(3), g.max(3), size(data_brs, 3));
        delta_value = delta_values(opts.deltaSlice);
    else
        brs_2d_slice = data_brs;
        target_2d_slice = data0;
        delta_value = 0;
    end

    % Convert grid values from radians to degrees for plotting
    xs1_deg = g.xs{1}(:,:,1) * 180/pi;  % Yaw rate (gamma) in degrees
    xs2_deg = g.xs{2}(:,:,1) * 180/pi;  % Sideslip angle (beta) in degrees

    % Plot BRS boundary
    [~, h_brs] = contour(xs2_deg, xs1_deg, brs_2d_slice, [0 0], 'r-', 'LineWidth', 2);
    hold on;

    % Plot target set
    [~, h_target] = contour(xs2_deg, xs1_deg, target_2d_slice, [0 0], 'g-', 'LineWidth', 2);

    % Plot full trajectory
    h_traj = plot(traj(2,:) * 180/pi, traj(1,:) * 180/pi, 'b-', 'LineWidth', 1.5);

    % Mark start and end points
    h_start = plot(traj(2,1) * 180/pi, traj(1,1) * 180/pi, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    h_end = plot(traj(2,end) * 180/pi, traj(1,end) * 180/pi, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

    % Current position marker (will be updated during animation)
    h_current = plot(traj(2,1) * 180/pi, traj(1,1) * 180/pi, 'ro', 'MarkerSize', 12, 'LineWidth', 2);

    % Add labels and legend
    grid on;
    xlabel('Sideslip Angle \beta (deg)', 'FontSize', 12);
    ylabel('Yaw Rate \gamma (deg/s)', 'FontSize', 12);
    if ndims(data_brs) == 3
        title(sprintf('State Space View (δ = %.1f°)', delta_value * 180/pi), 'FontSize', 14);
    else
        title('State Space View', 'FontSize', 14);
    end
    legend([h_brs, h_target, h_traj, h_start, h_end, h_current], ...
           'BRS Boundary', 'Target Set', 'Trajectory', 'Start', 'End', 'Current Position', ...
           'Location', 'best', 'FontSize', 10);

    % Set reasonable axis limits
    axis tight;
    ax_limits = axis();
    axis_margin = 0.1 * max(ax_limits(2)-ax_limits(1), ax_limits(4)-ax_limits(3));
    axis([ax_limits(1)-axis_margin, ax_limits(2)+axis_margin, ...
          ax_limits(3)-axis_margin, ax_limits(4)+axis_margin]);

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

    % Create car shape
    car_color = [0.3, 0.5, 0.8];  % Light blue
    car_outline = [
        -rear_overhang, opts.carWidth/2;   % Rear left
        opts.wheelBase + front_overhang, opts.carWidth/2;   % Front left
        opts.wheelBase + front_overhang, -opts.carWidth/2;  % Front right
        -rear_overhang, -opts.carWidth/2;  % Rear right
        -rear_overhang, opts.carWidth/2    % Back to rear left (close the shape)
    ];

    % Create car shape
    h_car = patch('XData', car_outline(:,1), 'YData', car_outline(:,2), ...
                  'FaceColor', car_color, 'EdgeColor', 'k', 'LineWidth', 1.5);

    % Create wheels (rectangles)
    front_left_wheel = [
        opts.wheelBase, wheel_width/2;
        opts.wheelBase + wheel_length, wheel_width/2;
        opts.wheelBase + wheel_length, -wheel_width/2;
        opts.wheelBase, -wheel_width/2;
        opts.wheelBase, wheel_width/2
    ];

    front_right_wheel = front_left_wheel;
    front_right_wheel(:,2) = -front_left_wheel(:,2) - wheel_width;

    rear_left_wheel = front_left_wheel;
    rear_left_wheel(:,1) = rear_left_wheel(:,1) - opts.wheelBase;

    rear_right_wheel = front_right_wheel;
    rear_right_wheel(:,1) = rear_right_wheel(:,1) - opts.wheelBase;

    % Create wheel patches
    h_wheel_fl = patch('XData', front_left_wheel(:,1), 'YData', front_left_wheel(:,2) + opts.carWidth/2 - wheel_width/2, ...
                      'FaceColor', 'k', 'EdgeColor', 'k');
    h_wheel_fr = patch('XData', front_right_wheel(:,1), 'YData', front_right_wheel(:,2) - opts.carWidth/2 + wheel_width/2, ...
                      'FaceColor', 'k', 'EdgeColor', 'k');
    h_wheel_rl = patch('XData', rear_left_wheel(:,1), 'YData', rear_left_wheel(:,2) + opts.carWidth/2 - wheel_width/2, ...
                      'FaceColor', 'k', 'EdgeColor', 'k');
    h_wheel_rr = patch('XData', rear_right_wheel(:,1), 'YData', rear_right_wheel(:,2) - opts.carWidth/2 + wheel_width/2, ...
                      'FaceColor', 'k', 'EdgeColor', 'k');

    % Add direction indicator
    arrow_length = opts.carLength * 0.6;
    h_direction = quiver(0, 0, arrow_length, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);

    % Reference point (trajectory path)
    h_path = plot(0, 0, 'b-', 'LineWidth', 2);
    
    % Initialize path history arrays
    path_history_x = [];
    path_history_y = [];

    % Set up axes properties
    axis equal;
    grid on;
    xlabel('X Position (m)', 'FontSize', 12);
    ylabel('Y Position (m)', 'FontSize', 12);
    title('Car-Centric View', 'FontSize', 14);

    visible_grid_size_with_margin = visible_grid_size * 0.6;
    xlim([-visible_grid_size_with_margin/2, visible_grid_size_with_margin/2]);
    ylim([-visible_grid_size_with_margin/2, visible_grid_size_with_margin/2]);

    %% Create progress bar and time display
    progress_bar = uicontrol('Style', 'slider', ...
                            'Min', 1, 'Max', length(traj_tau), ...
                            'Value', 1, ...
                            'SliderStep', [1/length(traj_tau), 10/length(traj_tau)], ...
                            'Position', [200, 20, 1200, 20]);

    time_display = uicontrol('Style', 'text', ...
                             'String', sprintf('Time: %.2f s', traj_tau(1)), ...
                             'Position', [800, 45, 200, 20], ...
                             'BackgroundColor', 'white', ...
                             'FontSize', 12);

    % Add title to the figure
    sgtitle(sprintf('Car Trajectory Visualization (v_x = %.1f m/s)', vx), ...
            'FontSize', 16, 'FontWeight', 'bold');

    %% Animation
    fprintf('Starting animation...\n');

    % Calculate number of frames and frame times
    total_time = traj_tau(end) - traj_tau(1);
    if try_video
        % For video: create a specific number of evenly-spaced frames
        num_frames = ceil(total_time * opts.frameRate / opts.playSpeed);
        frame_times = linspace(traj_tau(1), traj_tau(end), num_frames);
    else
        % For display: just use the original time points
        frame_times = traj_tau;
        num_frames = length(frame_times);
    end

    % Animation loop
    for frame = 1:num_frames
        % Get the current time for this frame
        if try_video
            current_time = frame_times(frame);
            % Find the index in the original trajectory closest to current time
            [~, idx] = min(abs(traj_tau - current_time));
        else
            idx = frame;
            current_time = traj_tau(idx);
        end
        
        % Update progress bar and time display
        set(progress_bar, 'Value', idx);
        set(time_display, 'String', sprintf('Time: %.2f s', current_time));
        
        % Get current state
        current_gamma = gamma(idx);
        current_beta = beta(idx);
        current_delta = delta(idx);
        current_psi = psi(idx);
        current_x = x(idx);
        current_y = y(idx);
        
        % Update 3D state-space view
        set(h_current, 'XData', current_beta * 180/pi, 'YData', current_gamma * 180/pi);
        
        % Update top-down car-centric view
        % 1. Update grid position to create illusion of car movement
        [grid_x, grid_y] = meshgrid(-visible_grid_size/2:grid_spacing:visible_grid_size/2, ...
                                  -visible_grid_size/2:grid_spacing:visible_grid_size/2);
        
        % Translate grid points to car's frame
        translated_x = grid_x - mod(current_x, grid_spacing);
        translated_y = grid_y - mod(current_y, grid_spacing);
        
        % Rotate grid points around origin by -current_psi
        rotated_x = translated_x * cos(-current_psi) - translated_y * sin(-current_psi);
        rotated_y = translated_x * sin(-current_psi) + translated_y * cos(-current_psi);
        
        % Update grid points - convert to vectors for the plot function
        set(h_grid, 'XData', rotated_x(:), 'YData', rotated_y(:));
        set(h_grid_perp, 'XData', rotated_y(:), 'YData', rotated_x(:));
        
        % 2. Update wheel orientations for steering
        % Only rotate front wheels to match steering angle
        % Get the original wheel shapes
        fl_wheel_x = get(h_wheel_fl, 'XData');
        fl_wheel_y = get(h_wheel_fl, 'YData');
        fr_wheel_x = get(h_wheel_fr, 'XData');
        fr_wheel_y = get(h_wheel_fr, 'YData');
        
        % Calculate wheel center points
        fl_center_x = mean(front_left_wheel(:,1));
        fl_center_y = mean(front_left_wheel(:,2) + opts.carWidth/2 - wheel_width/2);
        fr_center_x = mean(front_right_wheel(:,1));
        fr_center_y = mean(front_right_wheel(:,2) - opts.carWidth/2 + wheel_width/2);
        
        % Rotate front wheels around their centers by the steering angle
        fl_x_offset = fl_wheel_x - fl_center_x;
        fl_y_offset = fl_wheel_y - fl_center_y;
        fr_x_offset = fr_wheel_x - fr_center_x;
        fr_y_offset = fr_wheel_y - fr_center_y;
        
        % Rotate offsets
        fl_rotated_x = fl_x_offset * cos(current_delta) - fl_y_offset * sin(current_delta);
        fl_rotated_y = fl_x_offset * sin(current_delta) + fl_y_offset * cos(current_delta);
        fr_rotated_x = fr_x_offset * cos(current_delta) - fr_y_offset * sin(current_delta);
        fr_rotated_y = fr_x_offset * sin(current_delta) + fr_y_offset * cos(current_delta);
        
        % Update wheel positions
        set(h_wheel_fl, 'XData', fl_center_x + fl_rotated_x, 'YData', fl_center_y + fl_rotated_y);
        set(h_wheel_fr, 'XData', fr_center_x + fr_rotated_x, 'YData', fr_center_y + fr_rotated_y);
        
        % 3. Update direction indicator
        set(h_direction, 'UData', arrow_length * cos(current_delta), ...
                         'VData', arrow_length * sin(current_delta));
        
        % 4. Update path trace (in car's reference frame)
        % Calculate the last 100 points of the path in car's reference frame
        look_back = 100;
        start_idx = max(1, idx - look_back);
        
        % Transform the world path to car's reference frame
        rel_path_x = [];
        rel_path_y = [];
        for i = start_idx:idx
            % Vector from current position to path point
            dx = x(i) - current_x;
            dy = y(i) - current_y;
            
            % Rotate to car's reference frame
            rx = dx * cos(-current_psi) - dy * sin(-current_psi);
            ry = dx * sin(-current_psi) + dy * cos(-current_psi);
            
            rel_path_x = [rel_path_x, rx];
            rel_path_y = [rel_path_y, ry];
        end
        
        % Update path
        set(h_path, 'XData', rel_path_x, 'YData', rel_path_y);
        
        % Ensure consistent axis limits
        axes(left_panel);
        axis equal;
        xlim([-visible_grid_size_with_margin/2, visible_grid_size_with_margin/2]);
        ylim([-visible_grid_size_with_margin/2, visible_grid_size_with_margin/2]);
        
        % Capture frame if recording video
        if try_video
            frame_data = getframe(fig);
            writeVideo(v, frame_data);
            
            % Show progress
            if mod(frame, 10) == 0
                fprintf('Recording: %.1f%% complete\n', 100 * frame / num_frames);
            end
        end
        
        % Update display
        drawnow;
        
        % Add a small pause if not recording to make animation smoother
        if ~try_video
            pause(0.01);
        end
    end

    % Close video file if recording
    if try_video
        close(v);
        fprintf('Video saved successfully to: %s\n', opts.videoFile);
    end

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
            'gridSize', opts.gridSize, ...
            'deltaSlice', opts.deltaSlice);
    end
end

end