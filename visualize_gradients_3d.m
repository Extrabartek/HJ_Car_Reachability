function visualize_gradients_3d(g, value_function_full, tau, varargin)
% VISUALIZE_GRADIENTS_3D Visualizes gradients of a value function in 3D
%
% This function creates a 3D visualization of gradients using arrows that 
% show the direction of optimal control at different points in state space.
% The gradient at each point is taken from the time when that point first
% enters the BRS (value function becomes non-positive).
%
% Inputs:
%   g                 - Grid structure from reachability analysis
%   value_function_full - Value function for all time steps
%   tau               - Time vector corresponding to value_function_full
%   varargin          - Optional parameters:
%                       'density'   - Sampling density (default: [10, 10, 5])
%                       'scale'     - Arrow scaling factor (default: 0.5)
%                       'slice'     - Slice plane index or value for filtering (optional)
%                       'sliceDim'  - Dimension to slice (default: 3)
%                       'normalize' - Whether to normalize arrows (default: true)
%                       'threshold' - Only show gradients where |value| < threshold
%                       'arrowColor' - Color for arrows (default: 'r')
%                       'showBoundary' - Whether to show BRS boundary (default: true)
%
% Example:
%   visualize_gradients_3d(g, data_brs_full, tau, 'density', [15, 15, 7]);

%% Parse inputs
p = inputParser;
p.addRequired('g', @isstruct);
p.addRequired('value_function_full', @isnumeric);
p.addRequired('tau', @isvector);
p.addParameter('density', [10, 10, 5], @isvector);
p.addParameter('scale', 0.5, @isscalar);
p.addParameter('slice', [], @isnumeric);
p.addParameter('sliceDim', 3, @(x) ismember(x, [1, 2, 3]));
p.addParameter('normalize', true, @islogical);
p.addParameter('threshold', inf, @isscalar);
p.addParameter('arrowColor', 'r', @(x) ischar(x) || isnumeric(x) && length(x) == 3);
p.addParameter('showBoundary', true, @islogical);

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

%% Create sample grid points
fprintf('Setting up visualization grid points...\n');

% Sample points along each dimension
sample_points = cell(dims, 1);
for i = 1:dims
    sample_points{i} = linspace(g.min(i), g.max(i), opts.density(i));
end

% Create meshgrid for visualization
[beta_mesh, gamma_mesh, delta_mesh] = meshgrid(sample_points{2}, sample_points{1}, sample_points{3});

% Convert to degrees for visualization
beta_deg = beta_mesh * 180/pi;    % beta (sideslip)
gamma_deg = gamma_mesh * 180/pi;  % gamma (yaw rate)
delta_deg = delta_mesh * 180/pi;  % delta (steering)

%% Initialize gradient arrays
gX = zeros(size(beta_mesh));  % Beta gradient component
gY = zeros(size(beta_mesh));  % Gamma gradient component
gZ = zeros(size(beta_mesh));  % Delta gradient component

%% Evaluate gradients at sampled points
fprintf('Evaluating gradients at sample points based on entry time...\n');

% Record which points are reachable
reachable_mask = false(size(beta_mesh));

% Evaluate gradients
total_points = numel(beta_mesh);
display_interval = max(1, floor(total_points / 20));  % Show progress every 5%
points_processed = 0;

for i = 1:total_points
    % Get point coordinates
    beta = beta_mesh(i);
    gamma = gamma_mesh(i);
    delta = delta_mesh(i);
    
    % Convert to state vector
    x = [gamma, beta, delta];
    
    % Interpolate to find entry time for this point
    entry_time_idx = eval_u(g, arrival_time_indices, x');
    
    % Skip points that never enter BRS
    if isinf(entry_time_idx) || isnan(entry_time_idx)
        continue;
    end
    
    % Convert to valid index (interpolation may give non-integer values)
    entry_time_idx = max(1, min(num_times, round(entry_time_idx)));
    
    % Get gradients at the entry time
    time_gradients = all_gradients{entry_time_idx};
    
    % Evaluate gradient at this point
    grad = eval_u(g, time_gradients, x');
    
    % Store gradient components (negated for visualization - showing direction of descent)
    gX(i) = -grad(2);  % beta gradient
    gY(i) = -grad(1);  % gamma gradient
    gZ(i) = -grad(3);  % delta gradient
    
    % Mark this point as reachable
    reachable_mask(i) = true;
    
    % Display progress
    points_processed = points_processed + 1;
    if mod(points_processed, display_interval) == 0 || i == total_points
        fprintf('Processed %d/%d points (%.1f%%)\n', points_processed, total_points, ...
            100 * points_processed / total_points);
    end
end

%% Apply threshold filter if specified
if ~isinf(opts.threshold)
    % Get final BRS for threshold
    final_brs = value_function_full(:,:,:,end);
    
    % Check each point against threshold
    for i = 1:total_points
        if ~reachable_mask(i)
            continue;  % Skip points not in BRS
        end
        
        % Get point coordinates
        beta = beta_mesh(i);
        gamma = gamma_mesh(i);
        delta = delta_mesh(i);
        
        % Evaluate value at this point in the final BRS
        val = eval_u(g, final_brs, [gamma, beta, delta]');
        
        % Filter based on threshold
        if abs(val) > opts.threshold
            reachable_mask(i) = false;
        end
    end
end

%% Apply slice filter if enabled
if ~isempty(opts.slice)
    slice_dim = opts.sliceDim;
    slice_value = opts.slice;
    
    % Convert index to value if needed
    if length(slice_value) == 1 && slice_value <= g.N(slice_dim)
        slice_value = g.vs{slice_dim}(slice_value);
    else
        % Convert from degrees to radians
        slice_value = slice_value * pi/180;
    end
    
    % Create mask based on slice dimension
    switch slice_dim
        case 1 % gamma (yaw rate)
            slice_mask = abs(gamma_mesh - slice_value) > 1e-4;
        case 2 % beta (sideslip)
            slice_mask = abs(beta_mesh - slice_value) > 1e-4;
        case 3 % delta (steering)
            slice_mask = abs(delta_mesh - slice_value) > 1e-4;
    end
    
    % Apply slice mask to reachability mask
    reachable_mask = reachable_mask & ~slice_mask;
end

%% Normalize gradients if requested
if opts.normalize
    % Compute magnitude
    magnitudes = sqrt(gX.^2 + gY.^2 + gZ.^2);
    
    % Find non-zero magnitudes
    valid_idx = magnitudes > 1e-10 & reachable_mask;
    
    % Normalize gradients (avoiding division by zero)
    gX(valid_idx) = gX(valid_idx) ./ magnitudes(valid_idx);
    gY(valid_idx) = gY(valid_idx) ./ magnitudes(valid_idx);
    gZ(valid_idx) = gZ(valid_idx) ./ magnitudes(valid_idx);
end

%% Scale gradients
gX(reachable_mask) = gX(reachable_mask) * opts.scale;
gY(reachable_mask) = gY(reachable_mask) * opts.scale;
gZ(reachable_mask) = gZ(reachable_mask) * opts.scale;

%% Create figure for visualization
figure;

%% Plot boundary of value function
if opts.showBoundary
    hold on;
    
    % Extract final BRS for visualization
    final_brs = value_function_full(:,:,:,end);
    
    % Create 3D grid for isosurface
    [beta_grid, gamma_grid, delta_grid] = meshgrid(...
        g.vs{2} * 180/pi,...  % beta (sideslip angle)
        g.vs{1} * 180/pi,...  % gamma (yaw rate)
        g.vs{3} * 180/pi);    % delta (steering angle)

    % Plot zero level set
    [faces, verts] = isosurface(beta_grid, gamma_grid, delta_grid, permute(final_brs, [2,1,3]), 0);
    if ~isempty(faces)
        p = patch('Faces', faces, 'Vertices', verts, ...
                'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.2);
    end
end

%% Plot gradients as arrows
% Extract only reachable points
X_plot = beta_deg(reachable_mask);
Y_plot = gamma_deg(reachable_mask);
Z_plot = delta_deg(reachable_mask);
gX_plot = gX(reachable_mask);
gY_plot = gY(reachable_mask);
gZ_plot = gZ(reachable_mask);

% Plot 3D arrows showing gradient directions
fprintf('Plotting %d gradient arrows...\n', numel(X_plot));
h_arrows = quiver3(X_plot, Y_plot, Z_plot, gX_plot, gY_plot, gZ_plot, 0, ...
                'Color', opts.arrowColor, 'LineWidth', 1.5);

%% Finalize the visualization
% Add axis labels
xlabel('Sideslip Angle (degrees)', 'FontSize', 12);
ylabel('Yaw Rate (degrees/s)', 'FontSize', 12);
zlabel('Steering Angle (degrees)', 'FontSize', 12);
title('Gradient Flow Field Based on BRS Entry Time', 'FontSize', 14);

% Add grid and set limits
grid on;
axis tight;
view(3);

% Enable 3D rotation
rotate3d on;

% Set up lighting for better visualization
lighting gouraud;
camlight('headlight');
material dull;

fprintf('Gradient visualization complete!\n');
end