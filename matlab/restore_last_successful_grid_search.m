function restore_last_successful_grid_search(buildModels)
%RESTORE_LAST_SUCCESSFUL_GRID_SEARCH Restore the verified pre-YOLO setup.
%
% This reproduces the configuration used by the successful Milestone 3 run
% recorded on 2026-08-12. It intentionally regenerates the models instead
% of modifying a model that may contain later manual URI/configuration edits.

if nargin < 1
    buildModels = true;
end

projectRoot = fileparts(fileparts(mfilename('fullpath')));
matlabDir = fullfile(projectRoot, 'matlab');
simulinkDir = fullfile(projectRoot, 'simulink');

bdclose('all');
cd(matlabDir);

% Exact network values from the last successful pre-YOLO grid-search run.
cfgFile = fullfile(matlabDir, 'config_autonomous.m');
cfgText = fileread(cfgFile);
cfgText = regexprep(cfgText, ...
    'cfg\.qdroneIp\s*=\s*''[^'']+'';', ...
    'cfg.qdroneIp = ''192.168.2.15'';');
fid = fopen(cfgFile, 'w');
assert(fid >= 0, 'Could not update %s.', cfgFile);
cleanup = onCleanup(@() fclose(fid));
fwrite(fid, cfgText, 'char');
clear cleanup;

clear build_milestone3_grid_search config_autonomous generateGridWaypoints;
rehash;

% Recreate both models from the original Quanser models. This also removes
% all later camera/YOLO and manual external-mode configuration experiments.
build_milestone3_grid_search(buildModels);

commanderFile = fullfile(simulinkDir, 'QDrone_Grid_Search_Commander.mdl');
missionFile = fullfile(simulinkDir, 'Mission_Server_QDrone_Grid_Search.mdl');
load_system(commanderFile);
load_system(missionFile);

commander = 'QDrone_Grid_Search_Commander';
mission = 'Mission_Server_QDrone_Grid_Search';

% Exact successful two-argument QUARC configuration. Port 17000 is derived
% automatically as the Target Manager URI; it is not passed as model URI.
set_param(commander, 'ExtModeMexArgs', ...
    ' ''-w -d /tmp -uri %u -URI_Host tcpip://192.168.2.11:18001'',''tcpip://192.168.2.15:17001?retries=10''');
set_param(commander, 'StopTime', 'inf');

% Exact single-QDrone Mission Server stream argument from the successful run.
set_param(mission, 'ExtModeMexArgs', ...
    '''-w -d %d -uri %u -URI_Follower tcpip://localhost:18001''');
set_param(mission, 'StopTime', 'inf');

save_system(commander, commanderFile);
save_system(mission, missionFile);

fprintf('\nRestored last successful pre-YOLO grid-search configuration.\n');
fprintf('Commander: %s\n', get_param(commander, 'ExtModeMexArgs'));
fprintf('Commander target: %s\n', get_param(commander, 'SystemTargetFile'));
fprintf('Commander step: %s s\n', get_param(commander, 'FixedStep'));
fprintf('Mission Server: %s\n', get_param(mission, 'ExtModeMexArgs'));
fprintf('\nRun from this directory: %s\n', matlabDir);
end
