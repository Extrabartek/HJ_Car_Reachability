function [trajectory_positions] = transformTrajectory(traj, traj_tau, vx, varargin)
% TRANSFORMTRAJECTORY Transforms trajectories from state space to physical space
%
% This function takes a trajectory in state space (yaw rate, side slip, steering angle)
% and transforms it to physical space (x position, y position, steering angle)
% by integrating the vehicle dynamics.
%
% Inputs:
%   traj        - State trajectory [gamma; beta; delta] over time
%                 gamma: yaw rate (rad/s)
%                 beta: side slip angle (rad)
%                 delta: steering angle (rad)
%   traj_tau    - Time points corresponding to trajectory
%   vx          - Constant longitudinal velocity (m/s)
%   varargin    - Optional parameter-value pairs:
%                 'x0'        - Initial x position (default: 0)
%                 'y0'        - Initial y position (default: 0)
%                 'psi0'      - Initial heading angle (default: 0)
%                 'visualize' - Whether to visualize result (default: false)
%
% Outputs:
%   trajectory_positions - Structure with the transformed trajectory:
%                        .x      - X positions
%                        .y      - Y positions
%                        .psi    - Heading angles (rad)
%                        .delta  - Steering angles (rad)
%                        .time   - Time vector
%                        .gamma  - Original yaw rate (rad/s)
%                        .beta   - Original side slip angle (rad)
%                        .vx     - Longitudinal velocities in vehicle frame
%                        .vy     - Lateral velocities in vehicle frame
%
% Example:
%   [pos_traj] = transformTrajectory(traj, traj_tau, 30, 'visualize', true);

%% Parse inputs
p = inputParser;
p.addRequired('traj', @isnumeric);
p.addRequired('traj_tau', @isnumeric);
p.addRequired('vx', @isnumeric);
p.addParameter('x0', 0, @isnumeric);
p.addParameter('y0', 0, @isnumeric);
p.addParameter('psi0', 0, @isnumeric);
p.addParameter('visualize', false, @islogical);

p.parse(traj, traj_tau, vx, varargin{:});
opts = p.Results;

% Initialize trajectory data structure
trajectory_positions = struct();
trajectory_positions.time = traj_tau;

% Check if traj has 3 states (yaw rate, side slip, steering angle)
if size(traj, 1) == 3
    gamma = traj(1, :); % yaw rate (rad/s)
    beta = traj(2, :);  % side slip angle (rad)
    delta = traj(3, :); % steering angle (rad)
elseif size(traj, 1) == 2
    % If only 2 states (yaw rate, side slip), assume steering angle is zero
    gamma = traj(1, :); % yaw rate (rad/s)
    beta = traj(2, :);  % side slip angle (rad)
    delta = zeros(size(gamma)); % steering angle (rad)
    warning('Only 2 states provided. Assuming steering angle is zero.');
else
    error('Input trajectory must have either 2 or 3 states.');
end

% Store original states
trajectory_positions.gamma = gamma;
trajectory_positions.beta = beta;
trajectory_positions.delta = delta;

% Initialize arrays for positions and heading
n = length(traj_tau);
x = zeros(1, n);
y = zeros(1, n);
psi = zeros(1, n);

% Set initial conditions
x(1) = opts.x0;
y(1) = opts.y0;
psi(1) = opts.psi0;

% Initialize velocity arrays
vx_global = zeros(1, n);
vy_global = zeros(1, n);
vx_body = zeros(1, n) + vx; % constant longitudinal velocity
vy_body = zeros(1, n);

% Compute lateral velocity from side slip angle
for i = 1:n
    % For small angles: vy ≈ vx * beta
    % More accurately: vy = vx * tan(beta)
    vy_body(i) = vx * tan(beta(i));
end

% Store body-frame velocities
trajectory_positions.vx = vx_body;
trajectory_positions.vy = vy_body;

% Compute heading by integrating yaw rate
for i = 2:n
    dt = traj_tau(i) - traj_tau(i-1);
    psi(i) = psi(i-1) + gamma(i-1) * dt;
    
    % Normalize angle to [-pi, pi]
    while psi(i) > pi
        psi(i) = psi(i) - 2*pi;
    end
    while psi(i) < -pi
        psi(i) = psi(i) + 2*pi;
    end
end

% Compute global velocities and positions
for i = 1:n
    % Transform velocities from body frame to global frame
    vx_global(i) = vx_body(i) * cos(psi(i)) - vy_body(i) * sin(psi(i));
    vy_global(i) = vx_body(i) * sin(psi(i)) + vy_body(i) * cos(psi(i));
    
    % Integrate to get positions
    if i > 1
        dt = traj_tau(i) - traj_tau(i-1);
        % Trapezoidal integration for higher accuracy
        x(i) = x(i-1) + 0.5 * (vx_global(i) + vx_global(i-1)) * dt;
        y(i) = y(i-1) + 0.5 * (vy_global(i) + vy_global(i-1)) * dt;
    end
end

% Store results
trajectory_positions.x = x;
trajectory_positions.y = y;
trajectory_positions.psi = psi;

% Visualize if requested
if opts.visualize
    figure('Name', 'Transformed Trajectory', 'Position', [100, 100, 1200, 600]);
    
    % Trajectory in physical space
    subplot(2, 3, [1, 4]);
    plot(x, y, 'b-', 'LineWidth', 2);
    hold on;
    plot(x(1), y(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Start
    plot(x(end), y(end), 'go', 'MarkerSize', 10, 'LineWidth', 2); % End
    
    % Plot vehicle orientation at regular intervals
    vehicle_indicators = 1:max(1, floor(n/20)):n;
    arrow_length = 0.5; % Adjust based on plot scale
    for i = vehicle_indicators
        % Draw vehicle orientation
        quiver(x(i), y(i), arrow_length * cos(psi(i)), arrow_length * sin(psi(i)), 0, 'k', 'LineWidth', 1.5);
        
        % Draw steering angle indicator (front wheels)
        wheel_angle = psi(i) + delta(i);
        quiver(x(i) + 0.25*cos(psi(i)), y(i) + 0.25*sin(psi(i)), ...
               0.2 * cos(wheel_angle), 0.2 * sin(wheel_angle), 0, 'r', 'LineWidth', 1);
    end
    
    grid on;
    axis equal;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Vehicle Trajectory');
    legend('Path', 'Start', 'End', 'Location', 'best');
    
    % Plot heading angle
    subplot(2, 3, 2);
    plot(traj_tau, unwrap(psi) * 180/pi, 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Heading Angle (deg)');
    title('Heading Angle vs Time');
    
    % Plot steering angle
    subplot(2, 3, 3);
    plot(traj_tau, delta * 180/pi, 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Steering Angle (deg)');
    title('Steering Angle vs Time');
    
    % Plot velocities
    subplot(2, 3, 5);
    plot(traj_tau, vx_body, 'b-', traj_tau, vy_body, 'r-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Vehicle-Frame Velocities');
    legend('v_x', 'v_y', 'Location', 'best');
    
    % Plot beta and gamma
    subplot(2, 3, 6);
    yyaxis left;
    plot(traj_tau, beta * 180/pi, 'b-', 'LineWidth', 2);
    ylabel('Side Slip Angle (deg)');
    
    yyaxis right;
    plot(traj_tau, gamma * 180/pi, 'r-', 'LineWidth', 2);
    ylabel('Yaw Rate (deg/s)');
    
    grid on;
    xlabel('Time (s)');
    title('Side Slip and Yaw Rate');
    legend('Side Slip', 'Yaw Rate', 'Location', 'best');
    
    % Adjust figure appearance
    set(gcf, 'Color', 'white');
    sgtitle(sprintf('Vehicle Trajectory (v_x = %.1f m/s)', vx), 'FontSize', 14, 'FontWeight', 'bold');
end

end