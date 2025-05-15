function verify_optimal_trajectory(traj, traj_tau, traj_u, trajectory_type, model_type, varargin)
% VERIFY_OPTIMAL_TRAJECTORY Verifies that a trajectory satisfies optimality principles
%
% For double integrator:
% - Check bang-bang structure (control should only switch once)
% - Verify control takes extreme values (acceleration limit)
%
% For Dubins car:
% - Check bang-bang structure 
% - Verify turning rate takes extreme values
%
% Inputs:
%   traj           - State trajectory
%   traj_tau       - Time points
%   traj_u         - Control inputs
%   trajectory_type - 'brs' or 'frs'
%   model_type     - 'doubleInt' or 'dubinsCar'
%   varargin       - Optional parameters

    %% Analyze control input structure
    fprintf('Analyzing control structure for %s trajectory (%s)...\n', ...
        upper(trajectory_type), model_type);
    
    % Check bang-bang property
    u_max = max(abs(traj_u));
    u_normalized = traj_u / u_max;
    
    % Find where control switches sign
    switches = find(diff(sign(u_normalized)) ~= 0);
    num_switches = length(switches);
    
    % Calculate time spent at extremes
    extreme_threshold = 0.95;  % Consider values above 95% of max as at extremes
    time_at_pos_extreme = sum(u_normalized > extreme_threshold) / length(u_normalized);
    time_at_neg_extreme = sum(u_normalized < -extreme_threshold) / length(u_normalized);
    time_at_extremes = time_at_pos_extreme + time_at_neg_extreme;
    
    % Check if model-specific properties are satisfied
    if strcmp(model_type, 'doubleInt')
        % For double integrator, expect at most one switch for time-optimal control
        fprintf('Number of control switches: %d (expected ≤ 1 for time-optimal control)\n', num_switches);
        if num_switches <= 1
            fprintf('✅ Control switch test passed\n');
        else
            fprintf('❌ Control switch test failed (too many switches)\n');
        end
    elseif strcmp(model_type, 'dubinsCar')
        % For Dubins car, expect at most 2 switches for time-optimal control
        fprintf('Number of control switches: %d (expected ≤ 2 for time-optimal control)\n', num_switches);
        if num_switches <= 2
            fprintf('✅ Control switch test passed\n');
        else
            fprintf('❌ Control switch test failed (too many switches)\n');
        end
    end
    
    % Check bang-bang property (control at extremes)
    fprintf('Time spent at control extremes: %.1f%%\n', time_at_extremes*100);
    if time_at_extremes > 0.9
        fprintf('✅ Bang-bang control test passed (> 90%% at extremes)\n');
    else
        fprintf('❌ Bang-bang control test failed (< 90%% at extremes)\n');
    end
    
    % Visualize control structure
    figure('Name', sprintf('%s Trajectory Control Analysis', upper(trajectory_type)));
    
    % Plot trajectory
    subplot(2,1,1);
    if strcmp(model_type, 'doubleInt')
        plot(traj_tau, traj(1,:), 'b-', 'LineWidth', 2);
        hold on;
        plot(traj_tau, traj(2,:), 'r-', 'LineWidth', 2);
        ylabel('State');
        legend('Position', 'Velocity');
    else
        plot(traj(1,:), traj(2,:), 'b-', 'LineWidth', 2);
        hold on;
        plot(traj(1,1), traj(2,1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
        plot(traj(1,end), traj(2,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        ylabel('Y Position');
        xlabel('X Position');
        legend('Trajectory', 'Start', 'End');
        axis equal;
    end
    title(sprintf('%s Trajectory (%s)', upper(trajectory_type), model_type));
    grid on;
    
    % Plot control signal
    subplot(2,1,2);
    plot(traj_tau(1:end-1), traj_u, 'k-', 'LineWidth', 2);
    hold on;
    
    % Highlight control switches
    if ~isempty(switches)
        plot(traj_tau(switches), traj_u(switches), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    end
    
    % Add optimal control lines
    plot(xlim(), [u_max, u_max], 'r--', 'LineWidth', 1);
    plot(xlim(), [-u_max, -u_max], 'r--', 'LineWidth', 1);
    
    ylabel('Control Input');
    xlabel('Time');
    title(sprintf('Control Input with %d Switches (%.1f%% at extremes)', num_switches, time_at_extremes*100));
    grid on;
end