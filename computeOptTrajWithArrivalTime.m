function [traj, traj_tau, traj_u] = computeOptTrajWithArrivalTime(g, data, tau, dynSys, extraArgs)
% COMPUTEOPTTRAJWITHARRIVALTIME Computes optimal trajectory using time-of-arrival function
%
% This function works similarly to computeOptTraj but uses the time-of-arrival function
% for gradient computation, which provides more robust directionality toward the target.

% Initial setup same as computeOptTraj
iter = 1;
tauLength = length(tau);
dtSmall = (tau(2) - tau(1))/4;  % Smaller time step for integration
maxIter = 1.25*tauLength;

% Initialize trajectory
traj = nan(g.dim, tauLength);
traj(:,1) = dynSys.x;
traj_u = zeros(1, tauLength-1);  % Store control inputs

% Get arrival time function
arrivalTime = extraArgs.arrivalTime;

% Main loop
while iter <= tauLength && iter <= maxIter
    % Current state
    x_current = traj(:,iter);
    
    % Compute gradients of arrival time function
    % Since arrival time DECREASES as we approach the target,
    % we need to negate the gradient to get the right direction
    deriv_arrival = computeGradients(g, arrivalTime);
    for i = 1:length(deriv_arrival)
        deriv_arrival{i} = -deriv_arrival{i};  % Negate gradient
    end
    
    % Evaluate gradient at current state
    deriv_at_x = eval_u(g, deriv_arrival, x_current);
    
    % Compute optimal control
    u = dynSys.optCtrl(tau(iter), x_current, deriv_at_x, extraArgs.uMode);
    
    % Store control
    if iter < tauLength
        traj_u(iter) = u;
    end
    
    % Check if target is reached
    arrivalVal = eval_u(g, arrivalTime, x_current);
    if arrivalVal <= tau(1) + 1e-6  % Close to initial time = reached target
        fprintf('Target reached at iter %d, time %.2f\n', iter, tau(iter));
        break;
    end
    
    % Update state with small time steps for accuracy
    for j = 1:4  % Subdivide time step for better accuracy
        deriv_at_x = eval_u(g, deriv_arrival, dynSys.x);
        u = dynSys.optCtrl(tau(iter), dynSys.x, deriv_at_x, extraArgs.uMode);
        dynSys.updateState(u, dtSmall, dynSys.x);
    end
    
    % Record new point on trajectory
    iter = iter + 1;
    if iter <= tauLength
        traj(:,iter) = dynSys.x;
    end
end

% Delete unused indices
traj(:,iter:end) = [];
traj_tau = tau(1:size(traj,2));
traj_u = traj_u(1:size(traj,2)-1);

end