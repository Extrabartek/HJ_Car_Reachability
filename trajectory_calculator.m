% Define folder paths (these should point to your actual result folders)
main_results_folder = '/home/bartosz/Documents/master_thesis/code_base/HJ_Car_Reachability/results/';
frs_folder = strcat(main_results_folder,'frs_results_20250401_175003_vx5-45_mz10000-10000');
brs_folder = strcat(main_results_folder,'brs_results_20250401_174533_vx5-45_mz10000-10000');

% Define an initial state (sideslip angle and yaw rate in radians)
xinit = [deg2rad(90); deg2rad(25)];

% Final state
xfinal = [0;0];

% Compute trajectory using both BRS and FRS with visualization
[traj, traj_tau, traj_u, metrics] = ...
compute_trajectory_from_folders(brs_folder, xinit, ...
    'frs_folder', frs_folder, ...
    'useFRS', false, ...
    'velocityIdx', 7, ...    % Use the 3rd velocity value
    'mzMaxIdx', 1, ...        % Use the 2nd Mzmax value
    'frsWeight', 0.0, ...     % Weight FRS constraints more heavily 
    'visualize', true, ...    % Show visualization
    'savePlots', false, ...
    'maxTime', 0.3);   % Save results