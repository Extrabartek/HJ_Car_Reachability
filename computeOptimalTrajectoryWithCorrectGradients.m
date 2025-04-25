function [traj, traj_tau, traj_u, traj_metrics] = computeOptimalTrajectoryWithCorrectGradients(g, data_brs_full, tau_brs, xinit, params, extra_args)
% COMPUTEOPTIMALTRAJECTORYWITHCORRECTGRADIENTS Computes optimal trajectory using correct time-of-arrival gradients
% with optimizations for performance

    % Ensure we have full time-varying BRS data
    if ndims(data_brs_full) < 4
        error('Full time-varying BRS data is required for correct gradient computation');
    end
    
    % Extract final time BRS for initial checks
    data_brs_final = data_brs_full(:,:,:,end);
    
    % Check if initial state is in BRS
    initial_value_brs = eval_u(g, data_brs_final, xinit);
    if initial_value_brs > 0
        error('Initial state is not in the BRS (value = %f)', initial_value_brs);
    end
    
    % Compute time-of-arrival function (when each state first enters the BRS)
    fprintf('Computing time-of-arrival function...\n');
    arrival_time = compute_arrival_time(data_brs_full, tau_brs);
    
    % OPTIMIZATION 1: Pre-compute all gradients for each time slice
    fprintf('Pre-computing gradients for all time slices...\n');
    num_time_slices = size(data_brs_full, 4);
    
    % Simple sequential approach - no parallel processing to avoid indexing issues
    all_gradients = cell(num_time_slices, 1);
    for t = 1:num_time_slices
        all_gradients{t} = computeGradients(g, data_brs_full(:,:,:,t));
        
        if mod(t, 10) == 0
            fprintf('Computed gradients for time slice %d/%d\n', t, num_time_slices);
        end
    end
    
    fprintf('Gradient pre-computation complete.\n');
    
    % OPTIMIZATION 2: Create fast lookup for mapping arrival time to index
    % Create simple lookup function - avoid interpolation for robustness
    tau_to_idx = @(t) min(max(1, find(tau_brs <= t, 1, 'last')), num_time_slices);
    
    % Initialize trajectory computation parameters
    dt = 0.001;  % Small time step for accurate integration
    max_time = extra_args.maxTime;
    if isempty(max_time)
        max_time = tau_brs(end);
    end
    max_steps = ceil(max_time / dt);
    
    % Initialize trajectory storage
    traj = zeros(3, max_steps+1);  % For 3D state [gamma; beta; delta]
    traj(:,1) = xinit;  % Set initial state
    traj_tau = (0:max_steps) * dt;  % Time vector
    traj_u = zeros(1, max_steps);  % Control inputs
    
    % Create vehicle model
    dCar = NonlinearBicycleSteered(xinit, params);
    
    % Main trajectory computation loop
    fprintf('Computing trajectory using correct time-of-arrival gradients...\n');
    
    for i = 1:max_steps
        % Current state and time
        x_current = traj(:,i);
        t_current = traj_tau(i);
        
        % Get arrival time for current state
        arrival_val = eval_u(g, arrival_time, x_current);
        
        % If state is not in BRS (arrival_val is inf), we're in trouble - stop computation
        if ~isfinite(arrival_val)
            warning('Current state at step %d is outside the BRS. Stopping computation.', i);
            % Truncate trajectory
            traj = traj(:, 1:i);
            traj_tau = traj_tau(1:i);
            traj_u = traj_u(1:i-1);
            break;
        end
        
        % Find the BRS time index corresponding to when this state entered the BRS
        brs_time_idx = tau_to_idx(arrival_val);
        
        % Use pre-computed gradients
        time_specific_gradients = all_gradients{brs_time_idx};
        
        % Evaluate gradient at current state
        deriv_at_x = eval_u(g, time_specific_gradients, x_current);
        
        % Compute optimal control
        u = dCar.optCtrl(t_current, x_current, deriv_at_x, extra_args.uMode);
        
        % Apply control limits
        dv_max = dCar.dv_max;
        if u > dv_max
            u = dv_max;
        elseif u < -dv_max
            u = -dv_max;
        end
        
        % Store control
        traj_u(i) = u;
        
        % Check if we've reached the target (very small arrival time)
        if arrival_val <= tau_brs(1) + dt
            fprintf('Target reached at step %d (time %.2fs)!\n', i, t_current);
            % Truncate trajectory
            traj = traj(:, 1:i);
            traj_tau = traj_tau(1:i);
            traj_u = traj_u(1:i-1);
            break;
        end
        
        % Propagate dynamics for one step
        dCar.updateState(u, dt);
        traj(:,i+1) = dCar.x;
        
        % Print progress occasionally
        if mod(i, 100) == 0
            fprintf('Step %d/%d (t = %.2fs): Arrival time = %.4fs\n', i, max_steps, t_current, arrival_val);
        end
    end
    
    % If loop completed without breaking, use full trajectory
    if i == max_steps
        fprintf('Reached maximum time without converging to target.\n');
        % Truncate any unused allocated space
        if i < size(traj, 2)
            traj = traj(:, 1:i);
            traj_tau = traj_tau(1:i);
            traj_u = traj_u(1:i-1);
        end
    end
    
    % Compute trajectory metrics
    traj_metrics = struct();
    
    % Time to reach target
    traj_metrics.time_to_target = traj_tau(end);
    
    % Compute peak values
    traj_metrics.max_gamma = max(abs(traj(1,:)));  % Max yaw rate
    traj_metrics.max_beta = max(abs(traj(2,:)));   % Max sideslip angle
    traj_metrics.max_delta = max(abs(traj(3,:)));  % Max steering angle
    traj_metrics.max_control = max(abs(traj_u));   % Max steering rate
    
    % Calculate control energy
    traj_metrics.control_energy = sum(traj_u.^2) * (traj_tau(2) - traj_tau(1));
    
    % Calculate final BRS value
    traj_metrics.final_brs_value = eval_u(g, data_brs_final, traj(:,end));
    
    % Calculate final arrival time
    traj_metrics.final_arrival_time = eval_u(g, arrival_time, traj(:,end));
    
    fprintf('Trajectory computation complete: %d steps, %.2f seconds.\n', length(traj_tau), traj_tau(end));
end