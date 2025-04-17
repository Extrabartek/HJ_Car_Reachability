function [trajectory_positions] = transformTrajectory(traj, traj_tau, vx, varargin)
% TRANSFORMTRAJECTORY Transforms trajectories from state space to physical space
%
% This function takes a trajectory in state space (yaw rate, side slip, steering angle)
% and transforms it to physical space (x position, y position, steering angle)
% by integrating the vehicle dynamics. The resulting trajectory will have its
% INITIAL velocity vector aligned with the x-axis, but is free to move in any
% direction afterward.

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

%% STEP 1: Calculate trajectory in the original coordinate frame
% Initialize arrays for positions and heading
n = length(traj_tau);
x_orig = zeros(1, n);
y_orig = zeros(1, n);
psi_orig = zeros(1, n);

% Set initial conditions
x_orig(1) = opts.x0;
y_orig(1) = opts.y0;
psi_orig(1) = opts.psi0;

% Initialize velocity arrays
vx_body = zeros(1, n) + vx; % constant longitudinal velocity
vy_body = zeros(1, n);

% Compute lateral velocity from side slip angle
for i = 1:n
    % More accurately: vy = vx * tan(beta)
    vy_body(i) = vx * tan(beta(i));
end

% Store body-frame velocities
trajectory_positions.vx = vx_body;
trajectory_positions.vy = vy_body;

% Compute heading by integrating yaw rate
for i = 2:n
    dt = traj_tau(i) - traj_tau(i-1);
    psi_orig(i) = psi_orig(i-1) + gamma(i-1) * dt;
    
    % Normalize angle to [-pi, pi]
    while psi_orig(i) > pi
        psi_orig(i) = psi_orig(i) - 2*pi;
    end
    while psi_orig(i) < -pi
        psi_orig(i) = psi_orig(i) + 2*pi;
    end
end

% Initialize global velocity arrays
vx_global_orig = zeros(1, n);
vy_global_orig = zeros(1, n);

% Compute global velocities and positions
for i = 1:n
    % Transform velocities from body frame to global frame
    vx_global_orig(i) = vx_body(i) * cos(psi_orig(i)) - vy_body(i) * sin(psi_orig(i));
    vy_global_orig(i) = vx_body(i) * sin(psi_orig(i)) + vy_body(i) * cos(psi_orig(i));
    
    % Integrate to get positions
    if i > 1
        dt = traj_tau(i) - traj_tau(i-1);
        % Trapezoidal integration for higher accuracy
        x_orig(i) = x_orig(i-1) + 0.5 * (vx_global_orig(i) + vx_global_orig(i-1)) * dt;
        y_orig(i) = y_orig(i-1) + 0.5 * (vy_global_orig(i) + vy_global_orig(i-1)) * dt;
    end
end

%% STEP 2: Calculate rotation angle to align INITIAL velocity with x-axis
% Calculate the initial velocity direction only
initial_velocity_angle = atan2(vy_global_orig(1), vx_global_orig(1));
rotation_angle = -initial_velocity_angle; % Rotate to align initial velocity with x-axis

%% STEP 3: Rotate the entire trajectory based on initial velocity alignment
% Initialize arrays for rotated positions and heading
x = zeros(1, n);
y = zeros(1, n);
psi = zeros(1, n);
vx_global = zeros(1, n);
vy_global = zeros(1, n);

% Apply rotation to the entire trajectory
for i = 1:n
    % Rotate positions - FIXED BUG IN THIS SECTION
    x(i) = x_orig(i) * cos(rotation_angle) - y_orig(i) * sin(rotation_angle);
    y(i) = x_orig(i) * sin(rotation_angle) + y_orig(i) * cos(rotation_angle);
    
    % Rotate velocities - FIXED BUG IN THIS SECTION
    vx_global(i) = vx_global_orig(i) * cos(rotation_angle) - vy_global_orig(i) * sin(rotation_angle);
    vy_global(i) = vx_global_orig(i) * sin(rotation_angle) + vy_global_orig(i) * cos(rotation_angle);
    
    % Adjust heading angle
    psi(i) = psi_orig(i) + rotation_angle;
    
    % Normalize angle to [-pi, pi]
    while psi(i) > pi
        psi(i) = psi(i) - 2*pi;
    end
    while psi(i) < -pi
        psi(i) = psi(i) + 2*pi;
    end
end

% Store results in rotated frame
trajectory_positions.x = x;
trajectory_positions.y = y;
trajectory_positions.psi = psi;
trajectory_positions.vx_global = vx_global;
trajectory_positions.vy_global = vy_global;
trajectory_positions.rotation_angle = rotation_angle;