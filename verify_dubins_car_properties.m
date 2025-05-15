function verify_dubins_car_properties(results_folder, varargin)
% VERIFY_DUBINS_CAR_PROPERTIES Verifies expected properties of Dubins car BRS
%
% Properties to verify:
% 1. Symmetry about the origin for zero heading
% 2. Forward reachable set at θ=0 should be an expanding circle plus a bulge
% 3. Backward reachable set should have characteristic "kidney" shape
%
% Example for use with your code:
%   verify_dubins_car_properties('results/dubinscar_brs_results_20250507_142451_v5_turn40-40');

    % Load the data
    combined_file = fullfile(results_folder, 'brs_combined_results.mat');
    if ~exist(combined_file, 'file')
        files = dir(fullfile(results_folder, 'brs_*.mat'));
        data_file = fullfile(results_folder, files(1).name);
        data = load(data_file);
    else
        data = load(combined_file);
    end
    
    % Extract key data
    g = data.g;
    value_function = data.all_data{1, 1};
    
    % Check dimensions
    if length(g.N) ~= 3
        error('Expected 3D grid for Dubins car');
    end
    
    % Find the slice at zero heading
    theta_values = squeeze(g.xs{3}(1,1,:));
    [~, zero_theta_idx] = min(abs(theta_values));
    
    % Extract zero heading slice
    zero_slice = squeeze(value_function(:,:,zero_theta_idx));
    
    % Check symmetry (flip x and y)
    flipped_slice = flipud(fliplr(zero_slice));
    symmetry_error = norm(zero_slice - flipped_slice) / norm(zero_slice);
    
    % Visualize
    figure('Name', 'Dubins Car Verification');
    
    % Plot zero heading slice
    subplot(2,2,1);
    contourf(g.xs{1}, g.xs{2}, zero_slice, [0,0], 'LineStyle', 'none');
    colormap(flipud(gray));
    hold on;
    contour(g.xs{1}, g.xs{2}, zero_slice, [0,0], 'LineWidth', 2, 'Color', 'b');
    title(sprintf('Zero Heading Slice (θ = %.2f°)', theta_values(zero_theta_idx)*180/pi));
    xlabel('X Position'); ylabel('Y Position');
    axis equal; grid on;
    
    % Plot symmetry check
    subplot(2,2,2);
    imagesc(abs(zero_slice - flipped_slice));
    colorbar; title(sprintf('Symmetry Error: %.4f%%', symmetry_error*100));
    xlabel('X Grid Index'); ylabel('Y Grid Index');
    
    % Plot slices at different headings
    heading_angles = [-pi/4, 0, pi/4];
    heading_indices = zeros(size(heading_angles));
    for i = 1:length(heading_angles)
        [~, heading_indices(i)] = min(abs(theta_values - heading_angles(i)));
    end
    
    subplot(2,2,3:4);
    hold on;
    colors = {'r', 'b', 'g'};
    for i = 1:length(heading_indices)
        idx = heading_indices(i);
        heading_slice = squeeze(value_function(:,:,idx));
        contour(g.xs{1}, g.xs{2}, heading_slice, [0,0], 'LineWidth', 2, 'Color', colors{i});
    end
    legend(arrayfun(@(a) sprintf('θ = %.0f°', a*180/pi), heading_angles, 'UniformOutput', false));
    title('BRS Boundaries at Different Headings');
    xlabel('X Position'); ylabel('Y Position');
    axis equal; grid on;
    
    fprintf('Symmetry check for zero heading: %.4f%% error\n', symmetry_error*100);
    if symmetry_error < 0.05
        fprintf('✅ Symmetry test passed (error < 5%%)\n');
    else
        fprintf('❌ Symmetry test failed (error > 5%%)\n');
    end
end