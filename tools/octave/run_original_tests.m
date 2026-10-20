function run_original_tests(root)
%RUN_ORIGINAL_TESTS Run the project's own MATLAB tests, unmodified, in Octave.
addpath(fullfile(root, 'matlab'));
addpath(fullfile(root, 'tools', 'octave', 'shims'), '-begin');
addpath(fullfile(root, 'tools', 'octave'));
test_phase9_landing_gate();
test_phase9_abort_diagnostics();
test_phase10_arbiter_sequence(root);
check_templates(root);
printf('ALL ORIGINAL MATLAB TESTS PASSED IN OCTAVE\n');
end
