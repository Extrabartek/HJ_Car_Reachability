function visualize_steering_gradients(g, value_function_full, tau, varargin)
% VISUALIZE_STEERING_GRADIENTS Visualizes the steering angle gradient component
% using the time each point enters the BRS
%
% This function creates a visualization focusing on the gradient component
% in the direction of the steering angle (delta), computed at the specific time
% each point enters the BRS - matching how trajectories are calculated.
%
% Inputs:
%   g                  - Grid structure from reachability analysis
%   value_function_full - Value function for all time steps
%   tau                - Time vector corresponding to value_function_full

%% Parse inputs
p = inputParser;
p.addRequired('g', @isstruct);
p.addRequired('value_function_full', @isnumeric);
p.addRequired('tau', @isvector);
p.addParameter('density', [15, 15], @isvector); % 2D slice density
p.addParameter('deltaSlice', 0, @isscalar);  % Delta slice in degrees
p.addParameter('scale', 0.5, @isscalar);
p.addParameter('colormap', 'coolwarm', @ischar); % 'coolwarm', 'jet', etc.
p.addParameter('showBoundary', true, @islogical);
p.addParameter('showArrows', true, @islogical);

p.parse(g, value_function_full, tau, varargin{:});
opts = p.Results;

%% Verify dimensions
dims = g.dim;
if dims ~= 3
    error('This function is currently implemented for 3D state spaces only');
end

% Ensure we have full time data
if ndims(value_function_full) ~= 4
    error('Value function must include all time steps (4D array for 3D state space)');
end

% Number of time steps
num_times = size(value_function_full, 4);
if num_times ~= length(tau)
    warning('Time vector length (%d) does not match value function time steps (%d)', ...
        length(tau), num_times);
end

%% Compute time-of-arrival function (when each point first enters the BRS)
fprintf('Computing time-of-arrival function...\n');

% Initialize with infinity (points never reaching BRS)
arrival_time_indices = inf(g.N');

% Determine when each point first enters the BRS (scan from end to beginning)
for t = num_times:-1:1
    % Get value function at this time
    value_t = value_function_full(:,:,:,t);
    
    % Find points that are in BRS at this time (value <= 0)
    in_brs_t = (value_t <= 0);
    
    % For points that haven't yet been assigned an arrival time, 
    % record this time if they're in the BRS
    unassigned = isinf(arrival_time_indices);
    new_points = in_brs_t & unassigned;
    
    % Assign arrival time for these points
    arrival_time_indices(new_points) = t;
    
    % Display progress
    if mod(t, 10) == 0 || t == 1
        fprintf('Computing entry time: processed time step %d/%d\n', num_times-t+1, num_times);
    end
end

%% Compute gradients for each time slice
fprintf('Computing gradients for all time slices...\n');
all_gradients = cell(num_times, 1);

for t = 1:num_times
    value_t = value_function_full(:,:,:,t);
    all_gradients{t} = computeGradients(g, value_t);
    
    % Display progress
    if mod(t, 10) == 0 || t == num_times
        fprintf('Computed gradients for time slice %d/%d\n', t, num_times);
    end
end

%% Setup custom colormap for delta gradient (blue-white-red)
if strcmpi(opts.colormap, 'coolwarm')
    % Create a blue-white-red colormap (negative to positive)
    n = 256;
    cmap = zeros(n, 3);
    % Blue to white
    cmap(1:n/2, 1) = linspace(0, 1, n/2);
    cmap(1:n/2, 2) = linspace(0, 1, n/2);
    cmap(1:n/2, 3) = linspace(1, 1, n/2);
    % White to red
    cmap(n/2+1:n, 1) = linspace(1, 1, n/2);
    cmap(n/2+1:n, 2) = linspace(1, 0, n/2);
    cmap(n/2+1:n, 3) = linspace(1, 0, n/2);
    colormap(cmap);
else
    colormap(opts.colormap);
end

%% Extract final time BRS for boundary
final_brs = value_function_full(:,:,:,end);

%% Convert delta slice from degrees to radians
delta_rad = opts.deltaSlice * pi/180;

% Find the closest slice index
delta_values = g.vs{3};
[~, slice_idx] = min(abs(delta_values - delta_rad));
actual_delta = delta_values(slice_idx) * 180/pi;
fprintf('Using delta slice at %.2f degrees (closest to requested %.2f degrees)\n', ...
    actual_delta, opts.deltaSlice);

%% Extract 2D slice of value function
% Extract the beta and gamma grids at the delta slice
beta_grid = g.xs{2}(:,:,slice_idx) * 180/pi;   % Convert to degrees
gamma_grid = g.xs{1}(:,:,slice_idx) * 180/pi;  % Convert to degrees

% Extract the value function slice
value_slice = squeeze(final_brs(:,:,slice_idx));

% Extract the time-of-arrival slice
arrival_time_slice = squeeze(arrival_time_indices(:,:,slice_idx));

%% Create a mask for unreachable points
reachable_mask = isfinite(arrival_time_slice);

%% Create 2D visualization
% Split into two subplots: one for value function, one for delta gradient
subplot(1, 2, 1);
[~, h_value] = contourf(beta_grid, gamma_grid, value_slice, 20);
hold on;
[~, h_brs] = contour(beta_grid, gamma_grid, value_slice, [0, 0], 'LineWidth', 2, 'Color', 'k');
colorbar;
title(sprintf('Value Function at δ = %.1f°', actual_delta), 'FontSize', 14);
xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
grid on;

% Initialize the delta gradient field
delta_gradient = zeros(size(value_slice));
delta_gradient(~reachable_mask) = NaN; % Mark unreachable points as NaN

%% Compute entry-time gradients for each point in the slice
fprintf('Computing entry-time gradients for visualization...\n');

% Loop through all points in the slice
for i = 1:size(beta_grid, 1)
    for j = 1:size(beta_grid, 2)
        % Skip points that never enter the BRS
        if ~reachable_mask(i, j)
            continue;
        end
        
        % Get the entry time index for this point
        entry_idx = arrival_time_slice(i, j);
        
        % Ensure valid index
        entry_idx = max(1, min(num_times, round(entry_idx)));
        
        % Get the gradient at this entry time
        entry_gradients = all_gradients{entry_idx};
        
        % Create state vector [gamma, beta, delta]
        x = [gamma_grid(i, j)*pi/180, beta_grid(i, j)*pi/180, delta_rad]';
        
        % Evaluate gradient at this point and time
        grad = eval_u(g, entry_gradients, x);
        
        % Store the delta gradient component
        delta_gradient(i, j) = grad(3);
    end
end

%% Plot the delta gradient component based on entry time
subplot(1, 2, 2);
[~, h_grad] = contourf(beta_grid, gamma_grid, delta_gradient, 20);
hold on;
[~, h_brs2] = contour(beta_grid, gamma_grid, value_slice, [0, 0], 'LineWidth', 2, 'Color', 'k');
c = colorbar;
title(c, '∂V/∂δ');
clim([-max(abs(delta_gradient(:)), [], 'omitnan'), max(abs(delta_gradient(:)), [], 'omitnan')]);  % Symmetric colorbar
title(sprintf('Entry-Time Steering Gradient (∂V/∂δ) at δ = %.1f°', actual_delta), 'FontSize', 14);
xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
grid on;

%% Add control arrows if requested
if opts.showArrows
    % Sample points for arrows
    [beta_samples, gamma_samples] = meshgrid(...
        linspace(min(beta_grid(:)), max(beta_grid(:)), opts.density(1)),...
        linspace(min(gamma_grid(:)), max(gamma_grid(:)), opts.density(2))...
    );
    
    % Evaluate delta gradient at sample points based on entry time
    delta_grad_samples = zeros(size(beta_samples));
    in_brs_mask = false(size(beta_samples));
    
    for i = 1:numel(beta_samples)
        % Convert to radians for evaluation
        beta_rad = beta_samples(i) * pi/180;
        gamma_rad = gamma_samples(i) * pi/180;
        
        % Create state vector [gamma, beta, delta]
        x = [gamma_rad, beta_rad, delta_rad]';
        
        % Check if point is in BRS
        value = eval_u(g, final_brs, x);
        if value > 0
            % Point is outside the BRS, skip
            continue;
        end
        
        % Get entry time for this point
        entry_time_idx = eval_u(g, arrival_time_indices, x);
        
        % Skip points that never enter BRS
        if isinf(entry_time_idx) || isnan(entry_time_idx)
            continue;
        end
        
        % Convert to valid index
        entry_time_idx = max(1, min(num_times, round(entry_time_idx)));
        
        % Get gradient at entry time
        entry_gradients = all_gradients{entry_time_idx};
        
        % Evaluate gradient at this point
        grad = eval_u(g, entry_gradients, x);
        
        % Store delta gradient component
        delta_grad_samples(i) = grad(3);
        
        % Mark as in BRS
        in_brs_mask(i) = true;
    end
    
    % Create arrows showing control direction
    % For steering control with uMode='min': negative gradient → positive control
    % Plot in the gradient subplot
    subplot(1, 2, 2);
    
    % Filter out points outside the BRS
    beta_valid = beta_samples(in_brs_mask);
    gamma_valid = gamma_samples(in_brs_mask);
    delta_grad_valid = delta_grad_samples(in_brs_mask);
    
    if ~isempty(beta_valid)
        % Scale arrow size based on gradient magnitude
        max_grad = max(abs(delta_grad_valid));
        norm_grad = delta_grad_valid / max_grad * opts.scale;
        
        % Create arrows with changing colors based on sign
        % Positive gradient (red) = negative control = steering left
        % Negative gradient (blue) = positive control = steering right
        quiver(beta_valid, gamma_valid, ...
              zeros(size(beta_valid)), -norm_grad, ...
              0, 'LineWidth', 1.5);
        
        % Add info text about control direction
        text(min(beta_grid(:))+1, max(gamma_grid(:))-5, ...
             'Blue = Steer Right (δ̇ > 0)', 'Color', 'blue', 'FontWeight', 'bold');
        text(min(beta_grid(:))+1, max(gamma_grid(:))+10, ...
             'Red = Steer Left (δ̇ < 0)', 'Color', 'red', 'FontWeight', 'bold');
    end
end

fprintf('Entry-time steering gradient visualization complete!\n');
end