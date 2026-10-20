function build_milestone3_grid_search(doBuild)
%BUILD_MILESTONE3_GRID_SEARCH Create the Phase 4 grid-search models.
% Run from MATLAB R2021b with QUARC installed.
%   build_milestone3_grid_search(false) creates and updates the models
%   build_milestone3_grid_search(true)  also invokes the QUARC builds

if nargin < 1
    doBuild = false;
end

% Allow the builder to be run repeatedly in the same MATLAB session.
generatedModels = {'Mission_Server_QDrone_Grid_Search', ...
    'QDrone_Grid_Search_Commander'};
for k = 1:numel(generatedModels)
    if bdIsLoaded(generatedModels{k})
        close_system(generatedModels{k}, 0);
    end
end

projectRoot = fileparts(fileparts(mfilename('fullpath')));
sourceDir = fullfile(projectRoot, 'simulink', 'source_originals');
modelDir = fullfile(projectRoot, 'simulink');
configDir = fullfile(projectRoot, 'config');

hostSource = fullfile(sourceDir, 'MS_LF_QBot2_QDrone_Joystick_FrSky.mdl');
commanderSource = fullfile(sourceDir, 'Commander_Stabilizer_QDrone.mdl');
hostTarget = fullfile(modelDir, 'Mission_Server_QDrone_Grid_Search.mdl');
commanderTarget = fullfile(modelDir, 'QDrone_Grid_Search_Commander.mdl');
calibrationFile = fullfile(configDir, 'Calibration_2026-08-11.cal');
rigidBodyFile = fullfile(configDir, 'RigidBody_01.tra');
cfg = config_autonomous();

assert(isfile(hostSource), 'Missing host source: %s', hostSource);
assert(isfile(commanderSource), 'Missing Commander source: %s', commanderSource);
assert(isfile(calibrationFile), 'Missing calibration file: %s', calibrationFile);
assert(isfile(rigidBodyFile), 'Missing rigid-body file: %s', rigidBodyFile);

createHost(hostSource, hostTarget, calibrationFile, rigidBodyFile, cfg.waypoints);
createCommander(commanderSource, commanderTarget);

updateModel('Mission_Server_QDrone_Grid_Search');
updateModel('QDrone_Grid_Search_Commander');

if doBuild
    slbuild('Mission_Server_QDrone_Grid_Search');
    slbuild('QDrone_Grid_Search_Commander');
end

save_system('Mission_Server_QDrone_Grid_Search');
save_system('QDrone_Grid_Search_Commander');
fprintf('\nMilestone 3 grid-search models created in:\n%s\n', modelDir);
end

function createHost(sourceFile, targetFile, calibrationFile, rigidBodyFile, gridWaypoints)
[~, sourceName] = fileparts(sourceFile);
removeShadowingPaths(sourceFile);
load_system(sourceFile);
save_system(sourceName, targetFile);
close_system(sourceName, 0);
model = 'Mission_Server_QDrone_Grid_Search';

mission = firstBlock(model, 'MISSION SERVER');

% Remove the top-level QBot communication execution path.
deleteBlocksContaining(model, {'Communication - QBot', 'Display1'});

% Remove QBot/follower command generation and QBot-only logging from the
% retained mission subsystem. Deleting a block also removes its lines.
deleteBlocksContaining(mission, {
    'QBot', ...
    'Generate Follower Trajectory', ...
    'Raw joystick to Linear and Angular Velocity Commands', ...
    'Triggered Subsystem1', ...
    'Subtract1', ...
    'Product1', ...
    'Scope2', ...
    'Mux1', ...
    'To Host File'});

% The original communication alarm OR'ed QDrone and QBot faults. Retain only
% the QDrone alarm after the QBot branch is removed.
deleteBlocksExact(mission, {'Logical Operator'});
qdroneCommIssue = firstBlock(mission, 'Product');
commDisplay = firstBlock(mission, 'Communication Issue');
% Depending on QUARC library-link resolution, this block can be reported as
% either Reference or SubSystem. Its name is stable, so do not constrain type.
commMessage = firstBlock(mission, 'Show Message on Host');
safeAddLine(mission, qdroneCommIssue, 1, commDisplay, 1);
safeAddLine(mission, qdroneCommIssue, 1, commMessage, 1);

% Remove the old three-way follower/custom/home selector. The fixed-hover
% desired pose comes directly from the retained Quanser home-pose capture.
deleteBlocksExact(mission, {'Switch','Mode switch','Custom Mode','Home Mode', ...
    'Enable','Disable','Enable switch','isTracking1','Mux','Constant'});

% FrSky trigger outputs remain active; only the unused raw joystick-axis
% output is explicitly terminated in this fixed-position milestone.
joystick = firstBlock(mission, 'JOYSTICK');
missionName = getfullname(mission);
unusedJoystick = add_block('simulink/Sinks/Terminator', ...
    [missionName '/Unused Joystick Axes'], 'MakeNameUnique', 'on');
safeAddLine(mission, joystick, 3, unusedJoystick, 1);

measurementConditioning = firstBlock(mission, 'Measurement Conditioning');
unusedCorrectedPose = add_block('simulink/Sinks/Terminator', ...
    [missionName '/Unused Corrected Pose Log'], 'MakeNameUnique', 'on');
safeAddLine(mission, measurementConditioning, 3, unusedCorrectedPose, 1);

% Permanent external-mode diagnostic for boundary calibration. This is
% connected directly to the same conditioned [x,y,z,yaw] signal sent to the
% Commander, so it cannot accidentally display raw Motive coordinates.
poseDisplay = add_block('simulink/Sinks/Display', ...
    [missionName '/Measured Pose Display [x y z yaw]'], ...
    'Position', [1030 205 1190 255], ...
    'Decimation', '1', ...
    'MakeNameUnique', 'off');
safeAddLine(mission, measurementConditioning, 1, poseDisplay, 1);

homePose = firstBlock(mission, 'Calculate Home Pose');
desiredOut = firstBlock(mission, 'Desired');

% Phase 4: hold home through takeoff, then visit a conservative 5-by-4
% serpentine grid at 0.70 m. Start at the endpoint nearest the takeoff pose.
waypointManager = add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [missionName '/Grid Search Manager'], ...
    'Position', [1030 105 1235 185]);
setWaypointManagerScript(waypointManager, gridWaypoints);
safeAddLine(mission, measurementConditioning, 1, waypointManager, 1);
safeAddLine(mission, homePose, 1, waypointManager, 2);
safeAddLine(mission, joystick, 1, waypointManager, 3);
safeAddLine(mission, waypointManager, 1, desiredOut, 1);

waypointDisplay = add_block('simulink/Sinks/Display', ...
    [missionName '/Grid Waypoint Index Display'], ...
    'Position', [1245 195 1325 225]);
safeAddLine(mission, waypointManager, 2, waypointDisplay, 1);

completeDisplay = add_block('simulink/Sinks/Display', ...
    [missionName '/Grid Complete Display'], ...
    'Position', [1245 235 1325 265]);
safeAddLine(mission, waypointManager, 3, completeDisplay, 1);

visitedDisplay = add_block('simulink/Sinks/Display', ...
    [missionName '/Visited Waypoint Count Display'], ...
    'Position', [1245 275 1325 305]);
safeAddLine(mission, waypointManager, 4, visitedDisplay, 1);

% The retained subsystem captures [startX,startY,startYaw] on the original
% arm trigger and substitutes only the desired altitude.
homeHeight = firstBlock(mission, 'Home height');
set_param(homeHeight, 'Value', '0.70');

vehicleId = firstBlock(mission, 'Vehicle ID - QDrone');
set_param(vehicleId, 'Value', '1');

dataServer = firstBlock(mission, 'Data Server');
opti = firstBlock(dataServer, 'OptiTrack Trackables');
set_param(opti, ...
    'calibration_file', calibrationFile, ...
    'rigid_body_file', rigidBodyFile, ...
    'rigid_body_ids', '1');
configureSingleBodyDataServer(dataServer);

% The retained raw tracking display originally selected element 14 from a
% two-body [position,orientation,tracking] vector. One body has seven values.
rawTrackingBlocks = find_system(mission, 'FindAll', 'on', 'SearchDepth', 1, ...
    'Type', 'Block', 'BlockType', 'Selector', 'Name', 'isTracking');
assert(~isempty(rawTrackingBlocks), 'Raw QDrone tracking Selector not found.');
rawTracking = rawTrackingBlocks(1);
set_param(rawTracking, 'InputPortWidth', '7', 'Indices', '[7]');

% Preserve the original packet ordering and change only the QDrone server
% argument from the follower port to the single-drone port expected by the
% Commander (tcpip port 18001).
set_param(model, 'ExtModeMexArgs', ...
    '''-w -d %d -uri %u -URI_Follower tcpip://localhost:18001''');

deleteDanglingLines(model);
save_system(model, targetFile);
end

function createCommander(sourceFile, targetFile)
[~, sourceName] = fileparts(sourceFile);
removeShadowingPaths(sourceFile);
load_system(sourceFile);
save_system(sourceName, targetFile);
close_system(sourceName, 0);
model = 'QDrone_Grid_Search_Commander';

% Preserve all Quanser controller, FSM, watchdog, battery, tracking,
% saturation, and motor/stabilizer subsystems. Only communication deployment
% parameters are made explicit for QDrone 192.168.2.15.
set_param(model, 'ExtModeMexArgs', ...
    ' ''-w -d /tmp -uri %u -URI_Host tcpip://192.168.2.11:18001'',''tcpip://192.168.2.15:17001?retries=10''');
save_system(model, targetFile);
end

function updateModel(model)
load_system(model);
set_param(model, 'SimulationCommand', 'update');
fprintf('Updated model: %s\n', model);
end

function setWaypointManagerScript(block, gridWaypoints)
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', getfullname(block));
assert(~isempty(chart), 'MATLAB Function chart was not created.');
assert(isequal(size(gridWaypoints), [20 3]), ...
    'Phase 4 configuration must generate a 20-by-3 waypoint matrix.');
waypointLiteral = mat2str(gridWaypoints, 6);
chart.Script = sprintf([ ...
    'function [desiredPose, waypointIndex, gridComplete, visitedCount] = fcn(measuredPose, homePose, triggers)\n' ...
    '%%#codegen\n' ...
    'persistent stage dwell takeoffWasLow reverseOrder complete visited sessionActive\n' ...
    'if isempty(stage)\n' ...
    '    stage = uint8(0); dwell = uint16(0); takeoffWasLow = true; reverseOrder = false;\n' ...
    '    complete = false; visited = uint8(0); sessionActive = false;\n' ...
    'end\n' ...
    'takeoff = triggers(2) > 0.5;\n' ...
    'if ~takeoff\n' ...
    '    stage = uint8(0); dwell = uint16(0); takeoffWasLow = true; reverseOrder = false; sessionActive = false;\n' ...
    'elseif ~sessionActive\n' ...
    '    complete = false; visited = uint8(0); sessionActive = true;\n' ...
    'end\n' ...
    'wps = %s;\n' ...
    'desiredPose = homePose;\n' ...
    'desiredPose(3) = 0.70;\n' ...
    'if takeoff && takeoffWasLow\n' ...
    '    eHome = measuredPose(1:3) - desiredPose(1:3);\n' ...
    '    if sqrt(sum(eHome.*eHome)) <= 0.12\n' ...
    '        dwell = dwell + uint16(1);\n' ...
    '        if dwell >= uint16(100)\n' ...
    '            dFirst = measuredPose(1:2) - wps(1,1:2);\n' ...
    '            dLast = measuredPose(1:2) - wps(20,1:2);\n' ...
    '            reverseOrder = sum(dLast.*dLast) < sum(dFirst.*dFirst);\n' ...
    '            stage = uint8(1); dwell = uint16(0); takeoffWasLow = false;\n' ...
    '        end\n' ...
    '    else\n' ...
    '        dwell = uint16(0);\n' ...
    '    end\n' ...
    'elseif takeoff && stage >= 1\n' ...
    '    idx = double(stage);\n' ...
    '    if reverseOrder, idx = 21.0 - idx; end\n' ...
    '    desiredPose(1:3) = wps(idx,:);\n' ...
    '    e = measuredPose(1:3) - desiredPose(1:3);\n' ...
    '    if sqrt(sum(e.*e)) <= 0.12\n' ...
    '        dwell = dwell + uint16(1);\n' ...
    '        if dwell >= uint16(100)\n' ...
    '            visited = stage;\n' ...
    '            if stage < uint8(20)\n' ...
    '                stage = stage + uint8(1);\n' ...
    '            else\n' ...
    '                complete = true;\n' ...
    '            end\n' ...
    '            dwell = uint16(0);\n' ...
    '        end\n' ...
    '    else\n' ...
    '        dwell = uint16(0);\n' ...
    '    end\n' ...
    'end\n' ...
    'waypointIndex = 0.0;\n' ...
    'if stage >= uint8(1)\n' ...
    '    waypointIndex = double(stage);\n' ...
    '    if reverseOrder, waypointIndex = 21.0 - waypointIndex; end\n' ...
    'end\n' ...
    'gridComplete = complete;\n' ...
    'visitedCount = double(visited);\n' ...
    'end\n'], waypointLiteral);
end

function configureSingleBodyDataServer(dataServer)
% OptiTrack outputs position[3], orientation[3], tracking[1] for one body.
% Keep Quanser's QDrone frame correction and remove the parallel QBot path.
deleteBlocksExact(dataServer, {'Selector','Selector1','Selector2', ...
    'Data Type Conversion2'});

qdronePosition = firstBlock(dataServer, 'Selector4');
qdroneOrientation = firstBlock(dataServer, 'Selector5');
qdroneTracking = firstBlock(dataServer, 'Selector3');
set_param(qdronePosition, 'InputPortWidth', '3', 'Indices', '[1:3]');
set_param(qdroneOrientation, 'InputPortWidth', '3', 'Indices', '[1:3]');
set_param(qdroneTracking, 'InputPortWidth', '1', 'Indices', '[1]');

qbotConvention = firstBlock(dataServer, 'QBot 2e');
delete_block(qbotConvention);

qdroneConvention = firstBlock(dataServer, 'Convention Correction');
trackingConversion = firstBlock(dataServer, 'Data Type Conversion3');
mux = find_system(dataServer, 'FindAll', 'on', 'SearchDepth', 1, ...
    'Type', 'Block', 'BlockType', 'Mux', 'Name', 'Mux');
assert(~isempty(mux), 'Localization Mux not found under %s.', dataServer);
mux = mux(1);

% Disconnect the former two-body destinations before reducing the Mux.
ports = get_param(mux, 'PortHandles');
for k = 1:numel(ports.Inport)
    line = get_param(ports.Inport(k), 'Line');
    if line ~= -1
        delete_line(line);
    end
end
set_param(mux, 'Inputs', '3');
safeAddLine(dataServer, qdroneConvention, 1, mux, 1);
safeAddLine(dataServer, qdroneConvention, 2, mux, 2);
safeAddLine(dataServer, trackingConversion, 1, mux, 3);
end

function block = firstBlock(system, nameFragment)
blocks = find_system(system, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'Block');
systemHandle = get_param(system, 'Handle');
blocks(blocks == systemHandle) = [];
match = false(size(blocks));
for k = 1:numel(blocks)
    match(k) = contains(get_param(blocks(k), 'Name'), nameFragment, 'IgnoreCase', true);
end
blocks = blocks(match);
assert(~isempty(blocks), 'Block containing "%s" not found under %s.', nameFragment, system);
block = blocks(1);
end

function deleteBlocksContaining(system, fragments)
blocks = find_system(system, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'Block');
systemHandle = get_param(system, 'Handle');
blocks(blocks == systemHandle) = [];
for k = numel(blocks):-1:1
    name = get_param(blocks(k), 'Name');
    remove = false;
    for j = 1:numel(fragments)
        remove = remove || contains(name, fragments{j}, 'IgnoreCase', true);
    end
    if remove
        delete_block(blocks(k));
    end
end
end

function deleteDanglingLines(model)
% Deleting inherited branches can leave graphical line stubs. Remove only
% lines lacking a valid source or destination; fully connected command and
% safety signals remain untouched.
lines = find_system(model, 'FindAll', 'on', 'Type', 'line');
for k = numel(lines):-1:1
    try
        source = get_param(lines(k), 'SrcBlockHandle');
        destinations = get_param(lines(k), 'DstBlockHandle');
        validDestinations = destinations(destinations ~= -1);
        if source == -1 || isempty(validDestinations)
            delete_line(lines(k));
        end
    catch exception
        % Removing a parent line can invalidate branch handles captured by
        % find_system. Ignore only that expected stale-handle condition.
        if ~contains(exception.message, 'Invalid Simulink object handle')
            rethrow(exception);
        end
    end
end
end

function removeShadowingPaths(sourceFile)
% A Quanser applications folder may already be on the MATLAB path with a
% model having the same root name. Remove only those shadowing directories;
% the bundled source is loaded explicitly by absolute path afterward.
[sourceDir, modelName] = fileparts(sourceFile);
hits = which([modelName '.mdl'], '-all');
if ischar(hits)
    hits = {hits};
end
for k = 1:numel(hits)
    hitDir = fileparts(hits{k});
    if ~strcmpi(hitDir, sourceDir) && contains([path pathsep], [hitDir pathsep], 'IgnoreCase', true)
        rmpath(hitDir);
        fprintf('Temporarily removed shadowing model folder from MATLAB path: %s\n', hitDir);
    end
end
end

function deleteBlocksExact(system, names)
for k = 1:numel(names)
    blocks = find_system(system, 'FindAll', 'on', 'SearchDepth', 1, ...
        'Type', 'Block', 'Name', names{k});
    for j = numel(blocks):-1:1
        delete_block(blocks(j));
    end
end
end

function safeAddLine(system, sourceBlock, sourcePort, destBlock, destPort)
destHandles = get_param(destBlock, 'PortHandles');
oldLine = get_param(destHandles.Inport(destPort), 'Line');
if oldLine ~= -1
    delete_line(oldLine);
end
sourceName = get_param(sourceBlock, 'Name');
destName = get_param(destBlock, 'Name');
systemName = getfullname(system);
add_line(systemName, sprintf('%s/%d', sourceName, sourcePort), ...
    sprintf('%s/%d', destName, destPort), 'autorouting', 'on');
end
