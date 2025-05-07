function compute_safe_set_wrapper()

main_results_folder = '/home/bartosz/Documents/master_thesis/code_base/HJ_Car_Reachability/results/';

brs_folder = fullfile(main_results_folder, 'steered_brs_results_20250501_095810_vx20-20_dvmax40-40');
frs_folder = fullfile(main_results_folder, 'steered_frs_results_20250501_094443_vx20-20_dvmax40-40');


results_folder = compute_safe_set(brs_folder, frs_folder, ...
    'velocity_idx', 1, 'control_idx', 1, 'visualize', true);



end

