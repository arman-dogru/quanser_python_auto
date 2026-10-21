function check_templates(root)
%CHECK_TEMPLATES Prove the code inside the Phase 10 model is the repository's
% phase*.txt templates with config_autonomous() substituted exactly as the
% build scripts do (mat2str precision 6 for waypoints, 8 otherwise).
addpath(fullfile(root, 'matlab'));
addpath(fullfile(root, 'tools', 'octave', 'shims'), '-begin');
cfg = config_autonomous();
charts = fullfile(root, 'docs', 'spec', 'Mission_Server_QDrone_YOLO_Autonomous_Land', 'charts');
t = fileread(fullfile(root, 'matlab', 'phase8_grid_manager_code.txt'));
t = strrep(t, '__WAYPOINTS__', mat2str(cfg.waypoints, 6));
t = strrep(t, '__MAP__', mat2str(cfg.pixelToWorldAtSearchHeight, 8));
t = strrep(t, '__OFFSET__', mat2str(cfg.cameraOffsetBody, 8));
t = strrep(t, '__CENTER__', mat2str(cfg.cameraOpticalCenterPixel, 8));
t = strrep(t, '__YAW__', mat2str(cfg.visionCalibrationYaw, 8));
t = strrep(t, '__MAXSTEP__', mat2str(cfg.centerMaxStep, 8));
t = strrep(t, '__MAXDISPLACEMENT__', mat2str(cfg.centerMaxDisplacement, 8));
t = strrep(t, '__XTOL__', mat2str(cfg.xLimits, 8));
t = strrep(t, '__YTOL__', mat2str(cfg.yLimits, 8));
pairs = {t, 'chart_23_Grid_Search_Manager.m'; ...
  fileread(fullfile(root, 'matlab', 'phase9_landing_gate_code.txt')), 'chart_82_Landing_Readiness_Gate.m'; ...
  fileread(fullfile(root, 'matlab', 'phase9_abort_diagnostic_code.txt')), 'chart_99_Abort_Diagnostic.m'; ...
  fileread(fullfile(root, 'matlab', 'phase10_autoland_arbiter_code.txt')), 'chart_114_Autonomous_LAND_Trigger_Arbiter.m'};
for k = 1:rows(pairs)
  a = normalize(pairs{k,1});
  b = normalize(fileread(fullfile(charts, pairs{k,2})));
  if ~strcmp(a, b)
    error('check_templates:mismatch', 'Model chart %s differs from its template', pairs{k,2});
  end
  printf('PASS: %s == template + config_autonomous()\n', pairs{k,2});
end
end

function s = normalize(s)
s = strrep(s, sprintf('\r'), '');
lines = strsplit(s, sprintf('\n'));
lines = cellfun(@(l) regexprep(l, '\s+$', ''), lines, 'UniformOutput', false);
lines = lines(~cellfun(@isempty, lines));
s = strjoin(lines, sprintf('\n'));
end
