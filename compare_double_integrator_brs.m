function compare_double_integrator_brs(results_folder)
% COMPARE_DOUBLE_INTEGRATOR_BRS Compares a numerically computed double integrator BRS
% against the analytical solution (including switching regions)
%
% Input:
%   results_folder - Path to folder containing BRS computation results

%% Validate input
if nargin < 1 || ~exist(results_folder, 'dir')
    error('Please provide a valid results folder path');
end

%% Load the BRS data
fprintf('Loading BRS data from %s...\n', results_folder);

% First try loading combined results
combined_file = fullfile(results_folder, 'brs_combined_results.mat');
if ~exist(combined_file, 'file')
    % Try to find any brs_*.mat file
    files = dir(fullfile(results_folder, 'brs_*.mat'));
    if isempty(files)
        error('No BRS data files found in the specified folder');
    end
    data_file = fullfile(results_folder, files(1).name);
    fprintf('Loading individual BRS file: %s\n', data_file);
    data = load(data_file);
else
    data = load(combined_file);
end

%% Extract necessary data
% Get grid information
g = data.g;

% Get control limit
if isfield(data, 'acceleration_limits')
    control_limit = data.acceleration_limits(1);
elseif isfield(data, 'sim_params') && isfield(data.sim_params, 'acceleration_limits')
    control_limit = data.sim_params.acceleration_limits(1);
else
    % Prompt user for control limit if not found
    control_limit = input('Control limit (umax) not found. Please enter the value: ');
end

% Get target radius
if isfield(data, 'targetSize')
    target_radius = data.targetSize(1);
elseif isfield(data, 'sim_params') && isfield(data.sim_params, 'target_size')
    target_radius = data.sim_params.target_size(1);
else
    % Try to estimate from data0
    if isfield(data, 'data0')
        % Find the maximum position with value <= 0 when velocity = 0
        vel_zero_idx = ceil(size(data.data0, 2)/2);  % Middle index for velocity
        position_values = data.data0(:, vel_zero_idx);
        zero_positions = find(position_values <= 0);
        if ~isempty(zero_positions)
            target_radius = max(abs(g.xs{1}(zero_positions)));
        else
            target_radius = 0.5;  % Default
            warning('Could not estimate target radius. Using default: 0.5');
        end
    else
        target_radius = 0.5;  % Default
        warning('Target radius not found in data. Using default: 0.5');
    end
end

% Get time horizon
if isfield(data, 'tau')
    time_horizon = data.tau(end);
else
    % Default or prompt
    time_horizon = 1.0;
    warning('Time horizon not found in data. Using default: 1.0 s');
end

% Get value function data
if isfield(data, 'all_data')
    % For combined results, use the first velocity and control limit
    value_function = data.all_data{1, 1};
elseif isfield(data, 'data')
    % For individual results, use the data field
    value_function = data.data;
else
    error('Value function data not found.');
end

% If data is 3D (includes time), use the final time slice
if ndims(value_function) == 3
    % Use final time
    value_function_slice = value_function(:,:,end);
    
    % Get corresponding time if available
    if isfield(data, 'tau')
        time_value = data.tau(end);
        fprintf('Using final time slice (t = %.2f s)\n', time_value);
    else
        fprintf('Using final time slice\n');
    end
else
    % Single time slice
    value_function_slice = value_function;
    fprintf('Using single time slice (final BRS)\n');
end

fprintf('Using control limit: %.2f, target radius: %.2f, time horizon: %.2f\n', ...
        control_limit, target_radius, time_horizon);

%% Create figure for visualization
figure('Name', 'Double Integrator BRS Comparison', 'Position', [100, 100, 1000, 700]);

% Plot the BRS using filled contour
contourf(g.xs{1}, g.xs{2}, value_function_slice', [0, 0], 'LineStyle', 'none');
colormap(flipud(gray)); % Dark area represents the BRS
hold on;

% Plot the BRS boundary (numerical solution)
contour(g.xs{1}, g.xs{2}, value_function_slice', [0, 0], 'LineWidth', 2, 'Color', 'b');

% Create a dense grid for plotting analytical boundary
% Use the grid ranges from the data
x_min = min(g.xs{1}(:));
x_max = max(g.xs{1}(:));
v_min = min(g.xs{2}(:));
v_max = max(g.xs{2}(:));

v_dense = linspace(v_min, v_max, 1000);

% Compute analytical boundaries
% For regions requiring no switching

% 1. For positive velocities: x = -v²/(2*umax) - r
v_pos_dense = v_dense(v_dense >= 0);
v_threshold = sqrt(2 * control_limit * time_horizon); % Velocity threshold for switching
v_pos_nosw = v_pos_dense(v_pos_dense >= v_threshold);

if ~isempty(v_pos_nosw)
    x_pos_nosw = -v_pos_nosw.^2 / (2 * control_limit) - target_radius;
    plot(x_pos_nosw, v_pos_nosw, 'r-', 'LineWidth', 2);
end

% 2. For negative velocities: x = v²/(2*umax) + r
v_neg_dense = v_dense(v_dense < 0);
v_neg_nosw = v_neg_dense(v_neg_dense <= -v_threshold);

if ~isempty(v_neg_nosw)
    x_neg_nosw = v_neg_nosw.^2 / (2 * control_limit) + target_radius;
    plot(x_neg_nosw, v_neg_nosw, 'r-', 'LineWidth', 2);
end

% 3. For the switching region - parametric representation
% Generate the region where control switching is optimal
num_points = 200;
switching_times = linspace(0, time_horizon, num_points);

% Arrays for storing boundary points
x_switch_pos = zeros(1, num_points);
v_switch_pos = zeros(1, num_points);
x_switch_neg = zeros(1, num_points);
v_switch_neg = zeros(1, num_points);

for i = 1:num_points
    t_switch = switching_times(i);
    
    % Switching from +umax to -umax (right side)
    v_switch = control_limit * t_switch;
    x_dist = 0.5 * control_limit * t_switch^2;
    remaining_time = time_horizon - t_switch;
    x_decel = v_switch * remaining_time - 0.5 * control_limit * remaining_time^2;
    
    x_switch_pos(i) = target_radius + x_dist + x_decel;
    v_switch_pos(i) = v_switch;
    
    % Switching from -umax to +umax (left side)
    v_switch = -control_limit * t_switch;
    x_dist = -0.5 * control_limit * t_switch^2;
    remaining_time = time_horizon - t_switch;
    x_accel = v_switch * remaining_time + 0.5 * control_limit * remaining_time^2;
    
    x_switch_neg(i) = -target_radius + x_dist + x_accel;
    v_switch_neg(i) = v_switch;
end

% Plot the switching regions
plot(x_switch_pos, v_switch_pos, 'r-', 'LineWidth', 2);
plot(x_switch_neg, v_switch_neg, 'r-', 'LineWidth', 2);

% Plot the target set
rectangle('Position', [-target_radius, -target_radius, 2*target_radius, 2*target_radius], ...
         'Curvature', [1, 1], 'LineWidth', 2, 'LineStyle', '--', 'EdgeColor', 'g');

% Add labels and legend
xlabel('Position', 'FontSize', 14);
ylabel('Velocity', 'FontSize', 14);
title(sprintf('Double Integrator BRS Comparison (u_{max} = %.2f, r = %.2f, T = %.2f)', ...
              control_limit, target_radius, time_horizon), 'FontSize', 16);
legend('BRS Region', 'Numerical Boundary', 'Analytical Boundary', '', '', 'Target Set', 'Location', 'best');
grid on;
axis equal;

% Optionally plot some example optimal trajectories
show_trajectories = true;
if show_trajectories
    % Choose some sample starting points inside and on the boundary
    
    % 1. A point requiring no switching (high positive velocity)
    x0_1 = -4;  % Far left position
    v0_1 = 5;   % High positive velocity
    
    % 2. A point requiring no switching (high negative velocity)
    x0_2 = 4;   % Far right position
    v0_2 = -5;  % High negative velocity
    
    % 3. A point requiring switching (near zero velocity)
    x0_3 = 2.5; % Position outside target
    v0_3 = 0;   % Zero velocity
    
    % Function to simulate a trajectory with bang-bang control
    simulate_traj = @(x0, v0) simulate_trajectory(x0, v0, control_limit, target_radius, time_horizon);
    
    % Simulate and plot trajectories
    [t1, x1, v1, u1] = simulate_traj(x0_1, v0_1);
    [t2, x2, v2, u2] = simulate_traj(x0_2, v0_2);
    [t3, x3, v3, u3] = simulate_traj(x0_3, v0_3);
    
    % Plot trajectories in the phase space
    plot(x1, v1, 'b-', 'LineWidth', 1.5);
    plot(x2, v2, 'm-', 'LineWidth', 1.5);
    plot(x3, v3, 'c-', 'LineWidth', 1.5);
    
    % Mark starting points
    plot(x0_1, v0_1, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    plot(x0_2, v0_2, 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
    plot(x0_3, v0_3, 'co', 'MarkerSize', 8, 'MarkerFaceColor', 'c');
    
    % Create a second figure for the controls
    figure('Name', 'Optimal Control Inputs', 'Position', [150, 150, 800, 600]);
    
    subplot(3,1,1);
    stairs(t1, u1, 'b', 'LineWidth', 1.5);
    ylabel('Control u');
    title('Trajectory 1 (No Switching - Deceleration)');
    grid on;
    ylim([-control_limit*1.1, control_limit*1.1]);
    
    subplot(3,1,2);
    stairs(t2, u2, 'm', 'LineWidth', 1.5);
    ylabel('Control u');
    title('Trajectory 2 (No Switching - Acceleration)');
    grid on;
    ylim([-control_limit*1.1, control_limit*1.1]);
    
    subplot(3,1,3);
    stairs(t3, u3, 'c', 'LineWidth', 1.5);
    ylabel('Control u');
    title('Trajectory 3 (Switching Control)');
    xlabel('Time [s]');
    grid on;
    ylim([-control_limit*1.1, control_limit*1.1]);
end

fprintf('Visualization complete.\n');
end

% Helper function to simulate a trajectory with optimal control
function [t, x, v, u] = simulate_trajectory(x0, v0, umax, target_radius, time_horizon)
    % Determine optimal control strategy
    
    % For free time-optimal control to the origin:
    % u = -sign(v + sign(x)*sqrt(2*umax*abs(x)))
    
    % But for fixed-time optimal control to reach the target, 
    % we need to solve to find when to switch
    
    % We'll simulate the system numerically with small time steps
    dt = 0.01;
    num_steps = ceil(time_horizon / dt);
    
    % Initialize arrays
    t = zeros(1, num_steps);
    x = zeros(1, num_steps);
    v = zeros(1, num_steps);
    u = zeros(1, num_steps);
    
    % Set initial conditions
    x(1) = x0;
    v(1) = v0;
    
    % If target already reached, just stay there
    if abs(x0) <= target_radius && abs(v0) <= 0.1
        t = [0, time_horizon];
        x = [x0, x0];
        v = [v0, v0];
        u = [0, 0];
        return;
    end
    
    % Determine if we need switching
    if v0 > 0
        % For positive velocity, check if braking is enough
        stop_distance = v0^2 / (2 * umax);
        if x0 + stop_distance + target_radius < 0
            % Need to switch controls
            needs_switching = true;
            initial_control = umax;  % First accelerate
        else
            % Just brake
            needs_switching = false;
            initial_control = -umax; % Just decelerate
        end
    elseif v0 < 0
        % For negative velocity, check if accelerating is enough
        stop_distance = v0^2 / (2 * umax);
        if x0 - stop_distance - target_radius > 0
            % Need to switch controls
            needs_switching = true;
            initial_control = -umax; % First accelerate
        else
            % Just accelerate
            needs_switching = false;
            initial_control = umax;  % Just accelerate
        end
    else
        % For zero velocity, always need to switch if outside target
        if abs(x0) > target_radius
            needs_switching = true;
            initial_control = sign(-x0) * umax; % Accelerate away from target
        else
            % Already in target
            needs_switching = false;
            initial_control = 0;
        end
    end
    
    % Implement simulation with bang-bang control
    switched = false;
    switching_time = 0;
    
    % Calculate analytical switching time if needed
    if needs_switching
        if v0 >= 0 && x0 < -target_radius
            % Starting with positive velocity, to the left of target
            % Need to find when to switch from acceleration to deceleration
            
            % Solve for tau:
            % x0 + v0*tau + 0.5*umax*tau^2 + (v0+umax*tau)*(T-tau) - 0.5*umax*(T-tau)^2 = -R
            
            % Simplified: x0 + v0*T + 0.5*umax*(2*tau*T - tau^2 - T^2) = -R
            
            a = -umax;
            b = umax * time_horizon;
            c = x0 + v0 * time_horizon - 0.5 * umax * time_horizon^2 + target_radius;
            
            % Solve quadratic: a*tau^2 + b*tau + c = 0
            discriminant = b^2 - 4*a*c;
            if discriminant >= 0
                tau1 = (-b + sqrt(discriminant)) / (2*a);
                tau2 = (-b - sqrt(discriminant)) / (2*a);
                
                % Choose valid tau (between 0 and T)
                if tau1 >= 0 && tau1 <= time_horizon
                    switching_time = tau1;
                elseif tau2 >= 0 && tau2 <= time_horizon
                    switching_time = tau2;
                end
            end
        elseif v0 <= 0 && x0 > target_radius
            % Starting with negative velocity, to the right of target
            % Need to find when to switch from acceleration to deceleration
            
            % Similar calculation but mirrored
            a = -umax;
            b = umax * time_horizon;
            c = -x0 - v0 * time_horizon - 0.5 * umax * time_horizon^2 + target_radius;
            
            discriminant = b^2 - 4*a*c;
            if discriminant >= 0
                tau1 = (-b + sqrt(discriminant)) / (2*a);
                tau2 = (-b - sqrt(discriminant)) / (2*a);
                
                if tau1 >= 0 && tau1 <= time_horizon
                    switching_time = tau1;
                elseif tau2 >= 0 && tau2 <= time_horizon
                    switching_time = tau2;
                end
            end
        end
    end
    
    % Run simulation
    for i = 1:num_steps-1
        t(i) = (i-1) * dt;
        
        % Determine control input
        if needs_switching && t(i) >= switching_time && ~switched
            u(i) = -initial_control; % Switch control
            switched = true;
        else
            u(i) = initial_control;
        end
        
        % Update state with simple Euler integration
        v(i+1) = v(i) + u(i) * dt;
        x(i+1) = x(i) + v(i) * dt;
        
        % Check if reached target (optional early termination)
        if abs(x(i+1)) <= target_radius && abs(v(i+1)) <= 0.1
            % Truncate arrays
            t = t(1:i+1);
            x = x(1:i+1);
            v = v(1:i+1);
            u = u(1:i+1);
            return;
        end
    end
    
    % Set final time point
    t(end) = time_horizon;
    
    % Set final control input
    if needs_switching && t(end-1) >= switching_time
        u(end) = -initial_control;
    else
        u(end) = initial_control;
    end
end