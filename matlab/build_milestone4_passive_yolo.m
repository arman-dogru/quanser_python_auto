function build_milestone4_passive_yolo(doBuild)
%BUILD_MILESTONE4_PASSIVE_YOLO Build the separate Phase 5 camera application.
% The verified grid Mission Server and Commander are deliberately unchanged.
if nargin < 1
    doBuild = false;
end

projectRoot = fileparts(fileparts(mfilename('fullpath')));
sourceFile = fullfile(projectRoot, 'simulink', 'source_originals', ...
    'Read_OpticalFlow_Record.slx');
targetFile = fullfile(projectRoot, 'simulink', ...
    'QDrone_Downward_Camera_Passive.slx');
sourceModel = 'Read_OpticalFlow_Record';
targetModel = 'QDrone_Downward_Camera_Passive';
assert(isfile(sourceFile), 'Missing camera source model: %s', sourceFile);

if bdIsLoaded(sourceModel), close_system(sourceModel, 0); end
if bdIsLoaded(targetModel), close_system(targetModel, 0); end
removeShadowingPaths(sourceFile);
load_system(sourceFile);
save_system(sourceModel, targetFile);
close_system(sourceModel, 0);
load_system(targetModel);

% Remove every RealSense acquisition/display branch. Only the Omnivision
% OV7251 optical-flow Video Capture and grayscale display remain.
deleteTopBlocksContaining(targetModel, {'RealSense', 'Video3D'});

videoCapture = firstTopBlock(targetModel, 'Video Capture');
opticalFeed = firstTopBlock(targetModel, 'Optical Flow Camera feed');
assert(~isempty(videoCapture) && ~isempty(opticalFeed), ...
    'The downward optical-flow camera path was not preserved.');

% Camera application uses its own external-mode port. Commander remains on
% 17001; this passive camera target is deployed to QDrone 192.168.2.15:17002.
set_param(targetModel, 'ExtModeMexArgs', ...
    '''-w -d /tmp -uri %u'', ''tcpip://192.168.2.15:17002?retries=10''');
deleteDanglingLines(targetModel);
set_param(targetModel, 'SimulationCommand', 'update');
save_system(targetModel, targetFile);
fprintf('Updated model: %s\n', targetModel);

if doBuild
    slbuild(targetModel);
end
save_system(targetModel);
fprintf('\nPhase 5 passive downward-camera model created in:\n%s\n', ...
    fileparts(targetFile));
end

function removeShadowingPaths(sourceFile)
% Remove only MATLAB-path folders containing a same-named camera model.
[sourceDir, modelName, extension] = fileparts(sourceFile);
hits = which([modelName extension], '-all');
if ischar(hits), hits = {hits}; end
for k = 1:numel(hits)
    hitDir = fileparts(hits{k});
    if ~strcmpi(hitDir, sourceDir) && ...
            contains([path pathsep], [hitDir pathsep], 'IgnoreCase', true)
        rmpath(hitDir);
        fprintf('Temporarily removed shadowing model folder from MATLAB path: %s\n', ...
            hitDir);
    end
end
end

function block = firstTopBlock(model, fragment)
blocks = find_system(model, 'SearchDepth', 1, 'Type', 'Block');
blocks(strcmp(blocks, model)) = [];
match = false(size(blocks));
for k = 1:numel(blocks)
    match(k) = contains(get_param(blocks{k}, 'Name'), fragment, ...
        'IgnoreCase', true);
end
blocks = blocks(match);
assert(~isempty(blocks), 'Block containing "%s" not found.', fragment);
block = blocks{1};
end

function deleteTopBlocksContaining(model, fragments)
blocks = find_system(model, 'SearchDepth', 1, 'Type', 'Block');
blocks(strcmp(blocks, model)) = [];
for k = numel(blocks):-1:1
    name = get_param(blocks{k}, 'Name');
    remove = false;
    for j = 1:numel(fragments)
        remove = remove || contains(name, fragments{j}, 'IgnoreCase', true);
    end
    if remove, delete_block(blocks{k}); end
end
end

function deleteDanglingLines(model)
lines = find_system(model, 'FindAll', 'on', 'Type', 'line');
for k = numel(lines):-1:1
    try
        source = get_param(lines(k), 'SrcBlockHandle');
        destinations = get_param(lines(k), 'DstBlockHandle');
        if source == -1 || isempty(destinations(destinations ~= -1))
            delete_line(lines(k));
        end
    catch exception
        if ~contains(exception.message, 'Invalid Simulink object handle')
            rethrow(exception);
        end
    end
end
end
