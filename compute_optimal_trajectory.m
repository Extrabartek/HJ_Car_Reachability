function [traj, traj_tau, traj_u, traj_metrics] = compute_optimal_trajectory(g, data_brs, tau_brs, xinit, params, varargin)
% COMPUTE_OPTIMAL_TRAJECTORY Computes optimal trajectory for NonlinearBicycle model
%
% Inputs:
%   g           - Grid structure
%   data_brs    - Backward reachable set data (value function from BRS computation)
%   tau_brs     - Time vector for BRS (ascending order)
%   xinit       - Initial state [beta; gamma] (sideslip angle and yaw rate)
%   params      - Vehicle parameters [m; Vx; Lf; Lr; Iz; mu; Mzmax; Mzmin; Cf; Cr]
%   varargin    - Optional parameters as name-value pairs:
%                 'finalSet'    - Value function defining target final set (optional)
%                 'uMode'       - Control mode ('min' for BRS, default)
%                 'visualize'   - Whether to visualize (default: true)
%                 'figNum'      - Figure number for visualization
%                 'saveVideo'   - Whether to save visualization as video (default: false)
%                 'videoFile'   - Filename for video
%                 'data_frs'    - Forward reachable set data (optional)
%                 'tau_frs'     - Time vector for FRS (optional)
%                 'useFRSConstraint' - Whether to use FRS as a constraint (default: false)
%                 'frsWeight'   - Weight for FRS constraint (0-1, default: 0.5)
%                 'finalSetWeight' - Weight for final set targeting (default: 0.7)
%                 'maxTime'     - Maximum time for trajectory computation (default: based on tau_brs)
%
% Outputs:
%   traj        - Optimal trajectory states [beta; gamma] over time
%   traj_tau    - Time points corresponding to trajectory
%   traj_u      - Control inputs along trajectory
%   traj_metrics- Structure with additional metrics (if requested)
%
% Example:
%   % Using only BRS data (automatic final set):
%   [traj, traj_tau, traj_u] = compute_optimal_trajectory(g, data_brs, tau_brs, [0.1; 0.2], params);
%
%   % Specifying a custom final set:
%   small_target = shapeCylinder(g, 3, [0; 0], 0.05);  % Small target around origin
%   [traj, traj_tau, traj_u] = compute_optimal_trajectory(g, data_brs, tau_brs, [0.1; 0.2], params, ...
%       'finalSet', small_target);

%% Parse inputs
p = inputParser;
p.addRequired('g', @isstruct);
p.addRequired('data_brs', @isnumeric);
p.addRequired('tau_brs', @isnumeric);
p.addRequired('xinit', @isnumeric);
p.addRequired('params', @isnumeric);
p.addParameter('finalSet', [], @(x) isempty(x) || isnumeric(x));
p.addParameter('uMode', 'min', @ischar);
p.addParameter('visualize', true, @islogical);
p.addParameter('figNum', [], @(x) isempty(x) || isnumeric(x));
p.addParameter('saveVideo', false, @islogical);
p.addParameter('videoFile', 'trajectory_animation.mp4', @ischar);
p.addParameter('projDim', [1, 1], @isnumeric);  % Default: visualize in 2D
p.addParameter('plotTarget', true, @islogical);
p.addParameter('data_frs', [], @(x) isempty(x) || isnumeric(x));
p.addParameter('tau_frs', [], @(x) isempty(x) || isnumeric(x));
p.addParameter('useFRSConstraint', false, @islogical);
p.addParameter('frsWeight', 0.5, @(x) isnumeric(x) && x >= 0 && x <= 1);
p.addParameter('finalSetWeight', 0.7, @(x) isnumeric(x) && x >= 0 && x <= 1);
p.addParameter('maxTime', [], @(x) isempty(x) || (isnumeric(x) && x > 0));

p.parse(g, data_brs, tau_brs, xinit, params, varargin{:});
opts = p.Results;

% Set max time if not specified
if isempty(opts.maxTime)
    opts.maxTime = tau_brs(end);
end

% Check if both FRS data and time vector are provided when useFRSConstraint is true
if opts.useFRSConstraint
    if isempty(opts.data_frs) || isempty(opts.tau_frs)
        error('Both data_frs and tau_frs must be provided when useFRSConstraint is true');
    end
end

%% Check that initial state is within the BRS (only if no final set is provided)
if isempty(opts.finalSet)
    initial_value_brs = eval_u(g, data_brs(:,:,end), xinit);
    if initial_value_brs > 0
        error(['Initial state [', num2str(xinit(1)), ', ', num2str(xinit(2)), '] is not in the BRS (value = ', num2str(initial_value_brs), ')']);
    end
    
    % Check if initial state is also within the FRS if provided
    if ~isempty(opts.data_frs)
        initial_value_frs = eval_u(g, opts.data_frs(:,:,end), xinit);
        if initial_value_frs > 0
            warning(['Initial state [', num2str(xinit(1)), ', ', num2str(xinit(2)), '] is not in the FRS (value = ', num2str(initial_value_frs), ')']);
        else
            disp(['Initial state is in both BRS and FRS with values: BRS=', num2str(initial_value_brs), ', FRS=', num2str(initial_value_frs)]);
        end
    end
end

%% Initialize NonlinearBicycle model
dCar = NonlinearBicycle(xinit, params);

%% Choose trajectory computation method based on whether a final set is specified
if isempty(opts.finalSet)
    % Standard BRS-based trajectory
    [traj, traj_tau, traj_u, traj_metrics] = computeBRSTrajectory(g, data_brs, tau_brs, dCar, opts);
else
    % Final set-targeted trajectory
    [traj, traj_tau, traj_u, traj_metrics] = computeTargetedSetTrajectory(g, data_brs, tau_brs, dCar, opts.finalSet, opts);
end

%% Create visualization if requested
if opts.visualize
    % Create figure
    if isempty(opts.figNum)
        fig = figure;
    else
        fig = figure(opts.figNum);
    end
    
    % Create subplots
    subplot(2,2,1);
    plot(traj_tau, traj(2,:) * 180/pi, 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Sideslip Angle (deg)');
    title('Sideslip Angle vs Time');
    
    subplot(2,2,2);
    plot(traj_tau, traj(1,:) * 180/pi, 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Yaw Rate (deg/s)');
    title('Yaw Rate vs Time');
    
    subplot(2,2,3);
    plot(traj(2,:) * 180/pi, traj(1,:) * 180/pi, 'b-', 'LineWidth', 2);
    hold on;
    plot(traj(2,1) * 180/pi, traj(1,1) * 180/pi, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(traj(2,end) * 180/pi, traj(1,end) * 180/pi, 'go', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Visualize the BRS boundary
    if opts.plotTarget
        [~, h_brs] = contour(g.xs{2} * 180/pi, g.xs{1} * 180/pi, data_brs(:,:,end), [0 0], 'r-', 'LineWidth', 2);
        
        % Add final set boundary if provided
        if ~isempty(opts.finalSet)
            [~, h_final] = contour(g.xs{2} * 180/pi, g.xs{1} * 180/pi, opts.finalSet, [0 0], 'm-', 'LineWidth', 2);
        end
        
        % Add FRS boundary if available
        if ~isempty(opts.data_frs)
            [~, h_frs] = contour(g.xs{2} * 180/pi, g.xs{1} * 180/pi, opts.data_frs(:,:,end), [0 0], 'g--', 'LineWidth', 2);
            
            % Add intersection of BRS and FRS if both are available
            intersection = max(data_brs(:,:,end), opts.data_frs(:,:,end));
            [~, h_int] = contour(g.xs{2} * 180/pi, g.xs{1} * 180/pi, intersection, [0 0], 'k-.', 'LineWidth', 2);
            
            if isempty(opts.finalSet)
                legend_items = {'Trajectory', 'Initial State', 'Final State', 'BRS Boundary', 'FRS Boundary', 'Intersection'};
            else
                legend_items = {'Trajectory', 'Initial State', 'Final State', 'BRS Boundary', 'Target Set', 'FRS Boundary', 'Intersection'};
            end
        else
            if isempty(opts.finalSet)
                legend_items = {'Trajectory', 'Initial State', 'Final State', 'BRS Boundary'};
            else
                legend_items = {'Trajectory', 'Initial State', 'Final State', 'BRS Boundary', 'Target Set'};
            end
        end
    else
        if isempty(opts.finalSet)
            legend_items = {'Trajectory', 'Initial State', 'Final State'};
        else
            legend_items = {'Trajectory', 'Initial State', 'Final State', 'Target Set'};
        end
    end
    
    grid on;
    xlabel('Sideslip Angle (deg)');
    ylabel('Yaw Rate (deg/s)');
    title('Phase Portrait');
    legend(legend_items);
    
    subplot(2,2,4);
    if ~isempty(traj_u)
        plot(traj_tau(1:end-1), traj_u/1000, 'LineWidth', 2);
        grid on;
        xlabel('Time (s)');
        ylabel('Control Input (kN·m)');
        title('Yaw Moment Control');
    end
    
    if isempty(opts.finalSet)
        title_str = ['Optimal Trajectory from [', num2str(xinit(1) * 180/pi, '%.1f'), '°, ', ...
            num2str(xinit(2) * 180/pi, '%.1f'), '°/s]'];
    else
        title_str = ['Optimal Trajectory from [', num2str(xinit(1) * 180/pi, '%.1f'), '°, ', ...
            num2str(xinit(2) * 180/pi, '%.1f'), '°/s] to Target Set'];
    end
    
    sgtitle(title_str);
end

%% Save video if requested
if opts.saveVideo && opts.visualize
    % Implement video saving logic if needed
    warning('Video saving not implemented in this version');
end
end

%% Function to compute trajectory based on BRS (original method)
function [traj, traj_tau, traj_u, traj_metrics] = computeBRSTrajectory(g, data_brs, tau_brs, dCar, opts)
    %% Reverse time for trajectory computation (for BRS)
    % Flip data time points so we start from the beginning of time
    dataTraj = flip(data_brs, 3);
    tau_reversed = flip(tau_brs);
    
    %% Setup extra arguments for trajectory computation
    TrajExtraArgs.uMode = opts.uMode;
    TrajExtraArgs.dMode = 'max';  % Default disturbance mode (if applicable)
    TrajExtraArgs.visualize = false;  % Turn off default visualization - we'll create our own
    TrajExtraArgs.projDim = opts.projDim;  % Projection dimensions for visualization
    
    if ~isempty(opts.figNum)
        TrajExtraArgs.fig_num = opts.figNum;
    end
    
    %% Create custom dynamics system if using FRS constraint
    if opts.useFRSConstraint && ~isempty(opts.data_frs)
        % Create a wrapper for the dynamics to incorporate FRS constraint
        dCar_orig = dCar; % Store original dynamics
        
        % Override the optCtrl method to consider both BRS and FRS
        dCar.optCtrl = @(t, x, deriv, uMode) optCtrlWithFRS(dCar_orig, t, x, deriv, uMode, g, opts.data_frs, opts.frsWeight);
    end
    
    %% Compute optimal trajectory
    [traj, traj_tau, extraOuts] = computeOptTraj(g, dataTraj, tau_reversed, dCar, TrajExtraArgs);
    
    %% Extract control inputs
    % Get trajectory length
    traj_length = size(traj, 2);
    
    % Initialize control inputs storage
    traj_u = zeros(1, traj_length-1);
    
    % Compute control at each trajectory point
    for i = 1:traj_length-1
        x_current = traj(:, i);
        
        % Get value function at current time and state
        t_idx = find(tau_reversed >= traj_tau(i), 1, 'first');
        if isempty(t_idx), t_idx = 1; end
        
        % Compute gradient of value function
        deriv = computeGradients(g, dataTraj(:,:,t_idx));
        deriv_at_x = eval_u(g, deriv, x_current);
        
        % Get optimal control
        if opts.useFRSConstraint && ~isempty(opts.data_frs)
            % Use our custom control function that considers both BRS and FRS
            traj_u(i) = optCtrlWithFRS(dCar, traj_tau(i), x_current, deriv_at_x, opts.uMode, g, opts.data_frs, opts.frsWeight);
        else
            % Use standard control function
            traj_u(i) = dCar.optCtrl(traj_tau(i), x_current, deriv_at_x, opts.uMode);
        end
    end
    
    %% Compute additional trajectory metrics
    traj_metrics = struct();
    
    % Time to reach target
    traj_metrics.time_to_target = traj_tau(end);
    
    % Compute peak values
    traj_metrics.max_beta = max(abs(traj(1,:)));
    traj_metrics.max_gamma = max(abs(traj(2,:)));
    traj_metrics.max_control = max(abs(traj_u));
    
    % Calculate control energy
    traj_metrics.control_energy = sum(traj_u.^2);
    
    % Check if trajectory stays within FRS if provided
    if ~isempty(opts.data_frs)
        traj_metrics.in_frs = true;
        traj_metrics.min_frs_value = inf;
        
        for i = 1:traj_length
            frs_value = eval_u(g, opts.data_frs(:,:,end), traj(:,i));
            traj_metrics.min_frs_value = min(traj_metrics.min_frs_value, frs_value);
            
            if frs_value > 0
                traj_metrics.in_frs = false;
                traj_metrics.frs_violation_time = traj_tau(i);
                break;
            end
        end
    end
end

%% Function to compute trajectory targeting a specific final set
function [traj, traj_tau, traj_u, traj_metrics] = computeTargetedSetTrajectory(g, data_brs, tau_brs, dCar, finalSet, opts)
    % For targeted trajectories, we'll use a different approach
    % We'll perform direct shooting with a custom dynamics and control law
    % that aims to reach the specified final set
    
    %% Setup parameters
    % dt = tau_brs(2) - tau_brs(1);  % Time step based on BRS time vector
    dt = 0.0001;
    max_time = opts.maxTime;       % Maximum trajectory time
    max_steps = ceil(max_time / dt);
    
    % Constants
    finalSetWeight = opts.finalSetWeight;
    
    %% Initialize trajectory
    traj = zeros(2, max_steps+1);
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
        if false
            % We've reached the final set - stop trajectory computation
            disp(['Final set reached at step ' num2str(i) ' (time ' num2str(t_current) ' seconds)']);
            % Truncate the trajectory
            traj = traj(:, 1:i);
            traj_tau = traj_tau(1:i);
            traj_u = traj_u(1:i-1);
            break;
        end
        
        % Evaluate BRS value function and gradient
        brs_value = eval_u(g, data_brs(:,:,end), x_current);
        deriv_brs = computeGradients(g, data_brs(:,:,end));
        deriv_brs_at_x = eval_u(g, deriv_brs, x_current);
        
        % Evaluate gradient of the final set at current state
        deriv_final_at_x = eval_u(g, finalSetGrad, x_current);
        
        % Get FRS value (if available)
        frs_active = false;
        if opts.useFRSConstraint && ~isempty(opts.data_frs)
            frs_value = eval_u(g, opts.data_frs(:,:,end), x_current);
            if frs_value > -0.1  % If close to FRS boundary
                frs_active = true;
            end
        end
        
        % Compute control input based on:
        % 1. Gradient of final set to drive toward the set
        % 2. BRS gradient to maintain reachability
        % 3. FRS constraint (if enabled and active)
        
        % Final set targeting control - choose control that minimizes the value function
        % This will drive the system toward the final set (where the value function is <= 0)
        u_final = dCar_orig.optCtrl(t_current, x_current, deriv_final_at_x, 'min');
        
        % BRS navigation term
        u_brs = dCar_orig.optCtrl(t_current, x_current, deriv_brs_at_x, opts.uMode);
        
        % Blend control based on distance to target and BRS value
        if false
            % We're inside the BRS, blend between BRS guidance and target reaching
            % The weight depends on how far we are from the final set
            target_weight = min(1, abs(finalSet_value) * finalSetWeight);
            u = (1 - target_weight) * u_brs + target_weight * u_final;
        else
            % Outside BRS, prioritize getting back to the BRS
            u = u_brs;
        end
        
        % Apply FRS constraint if active
        if false
            % Get FRS gradient
            deriv_frs = computeGradients(g, opts.data_frs(:,:,end));
            deriv_frs_at_x = eval_u(g, deriv_frs, x_current);
            
            % Get control that keeps us in FRS
            u_frs = dCar_orig.optCtrl(t_current, x_current, deriv_frs_at_x, 'max');
            
            % If FRS control conflicts with current control, blend them
            if sign(u) * sign(u_frs) < 0
                frs_blend = min(1, opts.frsWeight * (0.1 - frs_value)/0.1);
                u = (1 - frs_blend) * u + frs_blend * u_frs;
            end
        end
        
        % Apply control limits
        Mzmax = dCar_orig.Mzmax;
        Mzmin = dCar_orig.Mzmin;
        if u < Mzmin
            u = Mzmin;
        end

        if u > Mzmax
            u = Mzmax;
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
    traj_metrics.max_beta = max(abs(traj(1,:)));
    traj_metrics.max_gamma = max(abs(traj(2,:)));
    traj_metrics.max_control = max(abs(traj_u));
    
    % Calculate control energy
    traj_metrics.control_energy = sum(traj_u.^2);
    
    % Target reaching performance
    traj_metrics.reached_target = (traj_metrics.final_set_value <= 0);
    
    % Check if trajectory stays within FRS if provided
    if ~isempty(opts.data_frs)
        traj_metrics.in_frs = true;
        traj_metrics.min_frs_value = inf;
        
        for i = 1:size(traj, 2)
            frs_value = eval_u(g, opts.data_frs(:,:,end), traj(:,i));
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
    frs_value = eval_u(g, data_frs(:,:,end), x);
    if frs_value > 0
        u = u_brs;
        return;
    end
    
    % If we're close to the FRS boundary, we need to consider the FRS gradient
    if frs_value > -0.1
        % Compute gradient of FRS value function
        deriv_frs = computeGradients(g, data_frs(:,:,end));
        deriv_frs_at_x = eval_u(g, deriv_frs, x);
        
        % Get optimal control for staying inside FRS (maximizing FRS value)
        u_frs = dynSys.optCtrl(t, x, deriv_frs_at_x, 'max');
        
        % Check if the controls are in opposite directions
        if false
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