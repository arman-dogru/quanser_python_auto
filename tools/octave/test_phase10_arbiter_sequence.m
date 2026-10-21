function test_phase10_arbiter_sequence(root)
%TEST_PHASE10_ARBITER_SEQUENCE Octave re-run of the behavioural half of
% matlab/test_phase10_autoland_dry_run.m. The original builds an in-memory
% Simulink harness (FixedStepDiscrete, 0.1 s, 10 samples) around the arbiter
% script; Octave has no Simulink, so this steps the same script directly over
% the same 10 samples and applies the original's exact assertions. The
% structural (wiring) half is ported to Python in tests/test_spec.py.
codeFile = fullfile(root, 'matlab', 'phase10_autoland_arbiter_code.txt');
script = fileread(codeFile);
script = strrep(script, '= fcn(', '= phase10_arbiter_under_test(');
d = tempname(); mkdir(d);
fid = fopen(fullfile(d, 'phase10_arbiter_under_test.m'), 'w'); fwrite(fid, script); fclose(fid);
addpath(d, '-begin'); clear phase10_arbiter_under_test;
raw = [0 0 0 0; 1 1 0 1; 1 1 0 1; 1 1 0 1; ...
       1 1 0 1; 1 0 0 1; 1 1 0 1; 0 1 0 1; ...
       1 1 1 1; 1 1 1 1];
ready = [0;0;1;0;0;0;0;0;1;0];
commanded = zeros(10,4); latch = zeros(10,1);
for k = 1:10
  [c, l] = phase10_arbiter_under_test(raw(k,:)', ready(k));
  commanded(k,:) = c(:)'; latch(k) = l;
end
rmpath(d); confirm_recursive_rmdir(false); rmdir(d, 's');
assert(isequal(commanded(:,[1 3 4]), raw(:,[1 3 4])), ...
  'Arm, emergency or other operator trigger was changed at the arbiter.');
assert(isequal(commanded(:,2)', [0 1 0 0 0 0 1 1 0 0]), ...
  'AUTOLAND request did not latch/reset as designed.');
assert(isequal(double(latch(:))', [0 0 1 1 1 0 0 0 1 1]), ...
  'AUTOLAND latch state did not match the expected sequence.');
printf('PASS: no request while unready; request latches only on readiness.\n');
printf('PASS: manual LAND and disarm reset the latch.\n');
printf('PASS: arm, emergency and remaining controls pass through.\n');
end
