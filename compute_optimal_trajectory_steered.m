function [traj, traj_tau, traj_u, traj_metrics] = compute_optimal_trajectory_steered(g, data_brs, tau_brs, xinit, params, extra_args)
% COMPUTE_OPTIMAL_TRAJECTORY_STEERED Computes optimal trajectory for NonlinearBicycleSteered model
%
% Inputs:
%   g           - Grid structure
%   data_brs    - Backward reachable set data (value function from BRS computation)
%   tau_brs     - Time vector for BRS (ascending order)
%   xinit       - Initial state [gamma; beta; delta] (yaw rate, sideslip angle, steering angle)
%   params      - Vehicle parameters [m; Vx; Lf; Lr; Iz; mu; dv_max; Cf; Cr]
%   extra_args  - Structure with additional parameters:
%                 .finalSet     - Value function defining target final set (optional)
%                 .uMode        - Control mode ('min' for BRS, default)
%                 .visualize    - Whether to visualize (default: true)
%                 .figNum       - Figure number for visualization
%                 .data_frs     - Forward reachable set data (optional)
%                 .tau_frs      - Time vector for FRS (optional)
%                 .useFRSConstraint - Whether to use FRS as a constraint (default: false)
%                 .frsWeight    - Weight for FRS constraint (0-1, default: 0.5)
%                 .finalSetWeight - Weight for final set targeting (default: 0.7)
%                 .maxTime      - Maximum time for trajectory computation (default: based on tau_brs)
%                 .deltaSlice   - Which delta slice to visualize for 2D views (default: middle slice)
%
% Outputs:
%   traj        - Optimal trajectory states [gamma; beta; delta] over time
%   traj_tau    - Time points corresponding to trajectory
%   traj_u      - Control inputs (steering rate) along trajectory
%   traj_metrics- Structure with additional metrics
%
% Example:
%   [traj, traj_tau, traj_u] = compute_optimal_trajectory_steered(g, data_brs, tau_brs, xinit, params);

%% Set default parameters if not provided

% Default control mode
if ~isfield(extra_args, 'uMode')
    extra_args.uMode = 'min';
end

% Default visualization setting
if ~isfield(extra_args, 'visualize')
    extra_args.visualize = true;
end

% Default maximum time
if ~isfield(extra_args, 'maxTime')
    extra_args.maxTime = tau_brs(end);
end

% Default FRS constraint setting
if ~isfield(extra_args, 'useFRSConstraint')
    extra_args.useFRSConstraint = false;
end

% Default FRS weight
if ~isfield(extra_args, 'frsWeight')
    extra_args.frsWeight = 0.5;
end

% Default final set weight
if ~isfield(extra_args, 'finalSetWeight')
    extra_args.finalSetWeight = 0.7;
end

% Default delta slice (middle slice)
if ~isfield(extra_args, 'deltaSlice')
    extra_args.deltaSlice = ceil(g.N(3)/2);
else
    if isempty(extra_args.deltaSlice)
        extra_args.deltaSlice = ceil(g.N(3)/2);
    end
end

%% Initialize NonlinearBicycleSteered model
dCar = NonlinearBicycleSteered(xinit, params);

%% Choose trajectory computation method based on whether a final set is specified

[traj, traj_tau, traj_u, traj_metrics] = computeBRSTrajectory(g, data_brs, tau_brs, dCar, extra_args);
%% Create visualization if requested
if extra_args.visualize
    % Create figure
    if isfield(extra_args, 'figNum') && ~isempty(extra_args.figNum)
        fig = figure(extra_args.figNum);
    else
        fig = figure;
    end
    
    % Extract delta slice for 2D visualizations
    delta_slice = extra_args.deltaSlice;
    if delta_slice < 1 || delta_slice > g.N(3)
        warning('Invalid delta slice index. Using middle slice instead.');
        delta_slice = ceil(g.N(3)/2);
    end
    
    % Get the delta value in radians for the selected slice
    delta_values = linspace(g.min(3), g.max(3), g.N(3));
    delta_value = delta_values(delta_slice);
    
    % Create a 3x3 grid of subplots
    % Top row: Yaw rate, Sideslip angle, Steering angle vs time
    % Middle row: Phase portraits
    % Bottom row: Control input, 3D visualization, Additional info
    
    % 1. Yaw rate vs time
    subplot(3, 3, 1);
    plot(traj_tau, traj(1,:) * 180/pi, 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Yaw Rate (deg/s)');
    title('Yaw Rate vs Time');
    
    % 2. Sideslip angle vs time
    subplot(3, 3, 2);
    plot(traj_tau, traj(2,:) * 180/pi, 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Sideslip Angle (deg)');
    title('Sideslip Angle vs Time');
    
    % 3. Steering angle vs time
    subplot(3, 3, 3);
    plot(traj_tau, traj(3,:) * 180/pi, 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Steering Angle (deg)');
    title('Steering Angle vs Time');
    
    % 4. Phase portrait: yaw rate vs sideslip
    subplot(3, 3, 4);
    plot(traj(2,:) * 180/pi, traj(1,:) * 180/pi, 'b-', 'LineWidth', 2);
    hold on;
    plot(traj(2,1) * 180/pi, traj(1,1) * 180/pi, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(traj(2,end) * 180/pi, traj(1,end) * 180/pi, 'go', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Extract and visualize the 2D slice of BRS at the specified steering angle
    brs_2d_slice = squeeze(data_brs(:,:,delta_slice));
    
    % Plot BRS slice
    [~, h_brs] = contour(g.xs{2}(:,:,1) * 180/pi, g.xs{1}(:,:,1) * 180/pi, brs_2d_slice, [0 0], 'r-', 'LineWidth', 2);
    
    % Add target set if provided
    if isfield(extra_args, 'finalSet') && ~isempty(extra_args.finalSet)
        target_2d_slice = squeeze(extra_args.finalSet(:,:,delta_slice));
        [~, h_target] = contour(g.xs{2}(:,:,1) * 180/pi, g.xs{1}(:,:,1) * 180/pi, target_2d_slice, [0 0], 'm-', 'LineWidth', 2);
        legend_items = {'Trajectory', 'Initial State', 'Final State', 'BRS Boundary', 'Target Set'};
    else
        legend_items = {'Trajectory', 'Initial State', 'Final State', 'BRS Boundary'};
    end
    
    grid on;
    xlabel('Sideslip Angle (deg)');
    ylabel('Yaw Rate (deg/s)');
    title(sprintf('Phase Portrait (δ = %.1f°)', delta_value * 180/pi));
    legend(legend_items);
    
    % 5. Phase portrait: steering angle vs sideslip
    subplot(3, 3, 5);
    plot(traj(2,:) * 180/pi, traj(3,:) * 180/pi, 'b-', 'LineWidth', 2);
    hold on;
    plot(traj(2,1) * 180/pi, traj(3,1) * 180/pi, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(traj(2,end) * 180/pi, traj(3,end) * 180/pi, 'go', 'MarkerSize', 10, 'LineWidth', 2);
    grid on;
    xlabel('Sideslip Angle (deg)');
    ylabel('Steering Angle (deg)');
    title('Steering Angle vs Sideslip');
    
    % 6. Phase portrait: steering angle vs yaw rate
    subplot(3, 3, 6);
    plot(traj(1,:) * 180/pi, traj(3,:) * 180/pi, 'b-', 'LineWidth', 2);
    hold on;
    plot(traj(1,1) * 180/pi, traj(3,1) * 180/pi, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(traj(1,end) * 180/pi, traj(3,end) * 180/pi, 'go', 'MarkerSize', 10, 'LineWidth', 2);
    grid on;
    xlabel('Yaw Rate (deg/s)');
    ylabel('Steering Angle (deg)');
    title('Steering Angle vs Yaw Rate');
    
    % 7. Control input
    subplot(3, 3, 7);
    if ~isempty(traj_u)
        plot(traj_tau(1:end-1), traj_u * 180/pi, 'LineWidth', 2);
        grid on;
        xlabel('Time (s)');
        ylabel('Steering Rate (deg/s)');
        title('Control Input (Steering Rate)');
    end
    
    % 8. 3D trajectory visualization with BRS and target set boundaries
subplot(3, 3, 8);
plot3(traj(2,:) * 180/pi, traj(1,:) * 180/pi, traj(3,:) * 180/pi, 'b-', 'LineWidth', 2);
hold on;
plot3(traj(2,1) * 180/pi, traj(1,1) * 180/pi, traj(3,1) * 180/pi, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot3(traj(2,end) * 180/pi, traj(1,end) * 180/pi, traj(3,end) * 180/pi, 'go', 'MarkerSize', 10, 'LineWidth', 2);

% Add BRS boundary as a 3D isosurface
if true
    % Extract zero level set of BRS
    [faces, verts] = isosurface(g.xs{2} * 180/pi, g.xs{1} * 180/pi, g.xs{3} * 180/pi, data_brs, 0);
    
    % Plot BRS isosurface as transparent red
    h_brs = patch('Faces', faces, 'Vertices', verts, 'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.2);
    
    % Add target set if provided
    if isfield(extra_args, 'finalSet') && ~isempty(extra_args.finalSet)
        % Extract zero level set of target set
        [faces_target, verts_target] = isosurface(g.xs{2} * 180/pi, g.xs{1} * 180/pi, g.xs{3} * 180/pi, extra_args.finalSet, 0);
        
        % Plot target set isosurface as transparent green
        h_target = patch('Faces', faces_target, 'Vertices', verts_target, 'FaceColor', 'g', 'EdgeColor', 'none', 'FaceAlpha', 0.2);
        
        % Update legend items for 3D plot
        legend('Trajectory', 'Initial State', 'Final State', 'BRS Boundary', 'Target Set', 'Location', 'best');
    else
        % Legend without target set
        legend('Trajectory', 'Initial State', 'Final State', 'BRS Boundary', 'Location', 'best');
    end
else
    % Simplified legend if no BRS visualization
    legend('Trajectory', 'Initial State', 'Final State', 'Location', 'best');
end

grid on;
xlabel('Sideslip Angle (deg)');
ylabel('Yaw Rate (deg/s)');
zlabel('Steering Angle (deg)');
title('3D Trajectory with BRS & Target');

% Adjust the view angle for better visualization
view([-30, 30]);

% Add lighting effects to enhance 3D visualization
lighting gouraud;
camlight;
material dull;
    
    % 9. Additional info (metrics display)
    subplot(3, 3, 9);
    % Create a text box with trajectory metrics
    metrics_text = {
        sprintf('Time to target: %.2f s', traj_metrics.time_to_target),
        sprintf('Max yaw rate: %.1f deg/s', traj_metrics.max_gamma * 180/pi),
        sprintf('Max sideslip angle: %.1f deg', traj_metrics.max_beta * 180/pi),
        sprintf('Max steering angle: %.1f deg', traj_metrics.max_delta * 180/pi),
        sprintf('Max steering rate: %.1f deg/s', traj_metrics.max_control * 180/pi),
        sprintf('Control energy: %.1e', traj_metrics.control_energy)
    };
    
    % Add FRS metric if available
    if isfield(traj_metrics, 'in_frs')
        if traj_metrics.in_frs
            metrics_text{end+1} = 'FRS constraint: Satisfied';
        else
            metrics_text{end+1} = sprintf('FRS violated at %.2f s', traj_metrics.frs_violation_time);
        end
    end
    
    % Add final set metric if available
    if isfield(traj_metrics, 'final_set_value')
        if traj_metrics.final_set_value <= 0
            metrics_text{end+1} = sprintf('Target reached (value: %.2e)', traj_metrics.final_set_value);
        else
            metrics_text{end+1} = sprintf('Target NOT reached (value: %.2e)', traj_metrics.final_set_value);
        end
    end
    
    % Turn off axes and display text
    axis off;
    text(0, 0.5, metrics_text, 'VerticalAlignment', 'middle');
    
    % Overall title for the figure
    if isfield(extra_args, 'finalSet') && ~isempty(extra_args.finalSet)
        title_str = ['Optimal Trajectory for Steered Bicycle (to Target Set)'];
    else
        title_str = ['Optimal Trajectory for Steered Bicycle'];
    end
    
    sgtitle(title_str, 'FontSize', 14, 'FontWeight', 'bold');
    
    % Adjust the figure size
    set(fig, 'Position', [100, 100, 1200, 900]);
end
end

function [traj, traj_tau, traj_u, traj_metrics] = computeBRSTrajectory(g, data_brs, tau_brs, dCar, extra_args)
    %% COMPUTEBRSTRAJECTORY Compute optimal trajectory based on BRS data
    % Uses either time-of-arrival function (preferred) or value function gradient
    
    % Setup integration parameters
    dt = 0.01;  % Smaller time step for accurate integration
    max_time = extra_args.maxTime;
    max_steps = ceil(max_time / dt);
    
    % Initialize trajectory storage
    traj = zeros(3, max_steps+1);  % For 3D state [gamma; beta; delta]
    traj(:,1) = dCar.x;  % Set initial state
    traj_tau = (0:max_steps) * dt;  % Time vector
    traj_u = zeros(1, max_steps);  % Control inputs
    
    % Create a working copy of the vehicle model
    dCar_orig = dCar;
    
    % Check if we have time-of-arrival data
    use_arrival_time = isfield(extra_args, 'arrival_time') && ...
                       ~isempty(extra_args.arrival_time);
                   
    if use_arrival_time
        fprintf('Using time-of-arrival function for trajectory computation.\n');
        arrival_time = extra_args.arrival_time;
        
        % Check if initial state is reachable according to arrival time
        arr_val = eval_u(g, arrival_time, dCar.x);
        if isfinite(arr_val)
            fprintf('Initial state is reachable in %.2f seconds (from arrival time).\n', arr_val);
        else
            warning('Initial state appears unreachable according to arrival time function!');
        end
        
        % Compute gradients of arrival time (only once, since it doesn't change with time)
        % Note: arrival time DECREASES toward the target, so we need to negate the gradient
        fprintf('Computing arrival time gradients...\n');
        arrival_grad = computeGradients(g, arrival_time);
        for i = 1:length(arrival_grad)
            arrival_grad{i} = -arrival_grad{i};  % Negate for correct direction
        end
    else
        fprintf('Using value function for trajectory computation.\n');
    end
    
    % Check for FRS constraint
    use_frs = isfield(extra_args, 'useFRSConstraint') && ...
              extra_args.useFRSConstraint && ...
              isfield(extra_args, 'data_frs') && ...
              ~isempty(extra_args.data_frs);
          
    if use_frs
        fprintf('Using FRS constraint with weight %.2f.\n', extra_args.frsWeight);
        data_frs = extra_args.data_frs;
    end
    
    %% Main trajectory computation loop
    fprintf('Computing trajectory...\n');
    for i = 1:max_steps
        % Current state and time
        x_current = traj(:,i);
        t_current = traj_tau(i);
        
        % Get gradient for control computation
        if use_arrival_time
            % Use arrival time gradient (already negated for correct direction)
            deriv_at_x = eval_u(g, arrival_grad, x_current);
        else
            % Use value function gradient
            deriv = computeGradients(g, data_brs);
            deriv_at_x = eval_u(g, deriv, x_current);
        end
        
        % Compute control
        
        u = dCar_orig.optCtrl(t_current, x_current, deriv_at_x, 'min');
        
        % Apply control limits
        dv_max = dCar_orig.dv_max;
        if u > dv_max
            u = dv_max;
        elseif u < -dv_max
            u = -dv_max;
        end
        
        % Store control
        traj_u(i) = u;
        
        % Check if we've reached the target
        if use_arrival_time
            arr_val = eval_u(g, arrival_time, x_current);
            if arr_val <= tau_brs(1) + dt  % Very close to initial time = target reached
                fprintf('Target reached at step %d (time %.2fs)!\n', i, t_current);
                % Truncate trajectory
                traj = traj(:, 1:i);
                traj_tau = traj_tau(1:i);
                traj_u = traj_u(1:i);
                break;
            end
        else
            % Check BRS value as proxy for target reaching
            brs_val = eval_u(g, data_brs, x_current);
            if brs_val < -0.95  % Deeply inside BRS, likely close to target
                fprintf('Likely reached target vicinity at step %d (time %.2fs).\n', i, t_current);
                % Continue trajectory a bit further to ensure target is reached
            end
        end
        
        % Check for divergence (far outside BRS)
        brs_val = eval_u(g, data_brs, x_current);
        if brs_val > 0.5  
            warning('Trajectory appears to be diverging (BRS value = %.2f). Stopping computation.', brs_val);
            % Truncate trajectory
            traj = traj(:, 1:i);
            traj_tau = traj_tau(1:i);
            traj_u = traj_u(1:i);
            break;
        end
        
        % Propagate dynamics for one step
        dCar_orig.updateState(u, dt);
        traj(:,i+1) = dCar_orig.x;
        
        % Print progress occasionally
        if mod(i, 100) == 0
            fprintf('Step %d/%d (t = %.2fs): BRS value = %.4f\n', i, max_steps, t_current, brs_val);
            if use_arrival_time
                arr_val = eval_u(g, arrival_time, x_current);
                if isfinite(arr_val)
                    fprintf('  Arrival time value = %.4fs\n', arr_val);
                else
                    fprintf('  Arrival time value = inf (outside reachable set)\n');
                end
            end
        end
    end
    
    % If loop completed without breaking, use full trajectory
    if i == max_steps
        fprintf('Reached maximum time without converging to target.\n');
    end
    
    %% Compute trajectory metrics
    traj_metrics = struct();
    
    % Time to reach target
    traj_metrics.time_to_target = traj_tau(end);
    
    % Compute peak values
    traj_metrics.max_gamma = max(abs(traj(1,:)));  % Max yaw rate
    traj_metrics.max_beta = max(abs(traj(2,:)));   % Max sideslip angle
    traj_metrics.max_delta = max(abs(traj(3,:)));  % Max steering angle
    traj_metrics.max_control = max(abs(traj_u));   % Max steering rate
    
    % Calculate control energy
    traj_metrics.control_energy = sum(traj_u.^2);
    
    % Check if trajectory stays within FRS if provided
    if use_frs
        traj_metrics.in_frs = true;
        traj_metrics.min_frs_value = inf;
        
        for j = 1:size(traj, 2)
            frs_value = eval_u(g, data_frs, traj(:,j));
            traj_metrics.min_frs_value = min(traj_metrics.min_frs_value, frs_value);
            
            if frs_value > 0
                traj_metrics.in_frs = false;
                traj_metrics.frs_violation_time = traj_tau(j);
                break;
            end
        end
        
        if traj_metrics.in_frs
            fprintf('Trajectory stays within FRS bounds (safe).\n');
        else
            fprintf('WARNING: Trajectory violates FRS bounds at %.2f seconds!\n', traj_metrics.frs_violation_time);
        end
    end
    
    fprintf('Trajectory computation complete: %d steps, %.2f seconds.\n', length(traj_tau), traj_tau(end));
end

%% Function to compute trajectory targeting a specific final set
function [traj, traj_tau, traj_u, traj_metrics] = computeTargetedSetTrajectory(g, data_brs, tau_brs, dCar, finalSet, extra_args)
    % For targeted trajectories, we'll use a different approach
    % We'll perform direct shooting with a custom dynamics and control law
    % that aims to reach the specified final set
    %% Setup parameters
    dt = 0.01;  % Time step for integration
    max_time = extra_args.maxTime;  % Maximum trajectory time
    max_steps = ceil(max_time / dt);

    
    % Constants
    finalSetWeight = extra_args.finalSetWeight;
    
    %% Initialize trajectory
    traj = zeros(3, max_steps+1);
    traj(:,1) = dCar.x;
    traj_tau = (0:max_steps) * dt;
    traj_u = zeros(1, max_steps);
    
    % Create a copy of the original vehicle model
    dCar_orig = dCar;
    
    % Compute gradient of the final set for guidance
    finalSetGrad = computeGradients(g, finalSet);
    
    %% Main trajectory computation loop
    for i = 1:max_steps
        % Current state
        x_current = traj(:,i);
        t_current = traj_tau(i);
        
        % Check if we've reached the final set
        finalSet_value = eval_u(g, finalSet, x_current);
        if finalSet_value <= 0
            % We've reached the final set - stop trajectory computation
            disp(['Final set reached at step ' num2str(i) ' (time ' num2str(t_current) ' seconds)']);
            % Truncate the trajectory
            traj = traj(:, 1:i);
            traj_tau = traj_tau(1:i);
            traj_u = traj_u(1:i-1);
            break;
        end
        
        % Evaluate BRS value function and gradient
        brs_value = eval_u(g, data_brs, x_current);
        deriv_brs = computeGradients(g, data_brs);
        deriv_brs_at_x = eval_u(g, deriv_brs, x_current);
        
        % Evaluate gradient of the final set at current state
        deriv_final_at_x = eval_u(g, finalSetGrad, x_current);
        
        % Get FRS value (if available)
        frs_active = false;
        if isfield(extra_args, 'useFRSConstraint') && extra_args.useFRSConstraint && isfield(extra_args, 'data_frs') && ~isempty(extra_args.data_frs)
            frs_value = eval_u(g, extra_args.data_frs, x_current);
            if frs_value > -0.1  % If close to FRS boundary
                frs_active = true;
            end
        end
        
        % Final set targeting control - choose control that minimizes the value function
        u_final = dCar_orig.optCtrl(t_current, x_current, deriv_final_at_x, 'min');
        
        % BRS navigation term
        % u_brs = dCar_orig.optCtrl(t_current, x_current, deriv_brs_at_x, extra_args.uMode);
        u_brs = dCar_orig.optCtrl(t_current, x_current, deriv_brs_at_x, 'min');
        
        % Blend control based on distance to target and BRS value
        u = u_brs;
        
        % Apply control limits
        dv_max = dCar_orig.dv_max;
        if u > dv_max
            u = dv_max;
        elseif u < -dv_max
            u = -dv_max;
        end
        
        % Store control input
        traj_u(i) = u;
        
        % Propagate dynamics for one step
        dCar_orig.updateState(u, dt);
        traj(:,i+1) = dCar_orig.x;
    end
    
    %% Compute trajectory metrics
    traj_metrics = struct();
    
    % Time to reach target
    traj_metrics.time_to_target = traj_tau(end);
    
    % Final set value at end of trajectory
    traj_metrics.final_set_value = eval_u(g, finalSet, traj(:,end));
    
    % Compute peak values
    traj_metrics.max_gamma = max(abs(traj(1,:)));  % Max yaw rate
    traj_metrics.max_beta = max(abs(traj(2,:)));   % Max sideslip angle
    traj_metrics.max_delta = max(abs(traj(3,:)));  % Max steering angle
    traj_metrics.max_control = max(abs(traj_u));   % Max steering rate
    
    % Calculate control energy
    traj_metrics.control_energy = sum(traj_u.^2);
    
    % Target reaching performance
    traj_metrics.reached_target = (traj_metrics.final_set_value <= 0);
    
    % Check if trajectory stays within FRS if provided
    if isfield(extra_args, 'useFRSConstraint') && extra_args.useFRSConstraint && isfield(extra_args, 'data_frs') && ~isempty(extra_args.data_frs)
        traj_metrics.in_frs = true;
        traj_metrics.min_frs_value = inf;
        
        for i = 1:size(traj, 2)
            frs_value = eval_u(g, extra_args.data_frs, traj(:,i));
            traj_metrics.min_frs_value = min(traj_metrics.min_frs_value, frs_value);
            
            if frs_value > 0
                traj_metrics.in_frs = false;
                traj_metrics.frs_violation_time = traj_tau(i);
                break;
            end
        end
    end
end

%% Helper function: Optimal control with FRS constraint
function u = optCtrlWithFRS(dynSys, t, x, deriv, uMode, g, data_frs, frsWeight)
% Custom control function that considers both BRS and FRS value functions
    
    % First, get the standard control from BRS
    u_brs = dynSys.optCtrl(t, x, deriv, uMode);
    
    % If the state is already outside the FRS, just use the BRS control
    frs_value = eval_u(g, data_frs, x);
    if frs_value > 0
        u = u_brs;
        return;
    end
    
    % If we're close to the FRS boundary, we need to consider the FRS gradient
    if frs_value > -0.1
        % Compute gradient of FRS value function
        deriv_frs = computeGradients(g, data_frs);
        deriv_frs_at_x = eval_u(g, deriv_frs, x);
        
        % Get optimal control for staying inside FRS (maximizing FRS value)
        u_frs = dynSys.optCtrl(t, x, deriv_frs_at_x, 'max');
        
        % Check if the controls are in opposite directions
        if sign(u_brs) * sign(u_frs) < 0
            % Controls are conflicting - use weighted average based on FRS value
            % The closer to the FRS boundary, the more we weight the FRS control
            boundary_weight = min(1, max(0, (0.1 - abs(frs_value))/0.1));
            combined_weight = frsWeight * boundary_weight;
            
            u = (1 - combined_weight) * u_brs + combined_weight * u_frs;
        else
            % Controls are compatible - use the BRS control
            u = u_brs;
        end
    else
        % We're comfortably inside the FRS - use the BRS control
        u = u_brs;
    end
end