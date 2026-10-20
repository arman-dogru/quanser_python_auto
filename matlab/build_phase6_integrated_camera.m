function build_phase6_integrated_camera(doBuild)
%BUILD_PHASE6_INTEGRATED_CAMERA Merge the downward camera into Commander.
% Creates a new Commander model and leaves all verified models untouched.
%
% The only imported camera path is:
%   Video Capture (device 2, grayscale) -> Optical Flow Camera feed
% No RealSense blocks, file recorder, second HIL path, or camera-model target
% settings are copied.

if nargin < 1
    doBuild = false;
end

projectRoot = fileparts(fileparts(mfilename('fullpath')));
modelDir = fullfile(projectRoot, 'simulink');
sourceCommanderFile = fullfile(modelDir, 'QDrone_YOLO_Interrupt_Commander.mdl');
sourceCameraFile = fullfile(modelDir, 'QDrone_Downward_Camera_Passive.slx');
targetFile = fullfile(modelDir, 'QDrone_YOLO_Integrated_Camera_Commander.mdl');
sourceCommander = 'QDrone_YOLO_Interrupt_Commander';
sourceCamera = 'QDrone_Downward_Camera_Passive';
targetModel = 'QDrone_YOLO_Integrated_Camera_Commander';

assert(isfile(sourceCommanderFile), ...
    'Missing verified YOLO Commander: %s', sourceCommanderFile);
assert(isfile(sourceCameraFile), ...
    'Missing passive camera model: %s', sourceCameraFile);

for name = {sourceCommander, sourceCamera, targetModel}
    if bdIsLoaded(name{1})
        close_system(name{1}, 0);
    end
end

% Clone the verified Commander under a new development name.
load_system(sourceCommanderFile);
save_system(sourceCommander, targetFile);
close_system(sourceCommander, 0);
load_system(targetFile);

% Open the validated passive model only as a source for the two camera
% blocks. Its target settings and other blocks are never copied.
load_system(sourceCameraFile);
captureSource = [sourceCamera '/Video Capture'];
displaySource = [sourceCamera '/Optical Flow Camera feed'];
assertBlockExists(captureSource);
assertBlockExists(displaySource);

captureDest = [targetModel '/Downward Grayscale Camera Capture'];
displayDest = [targetModel '/Optical Flow Camera feed'];
deleteIfPresent(captureDest);
deleteIfPresent(displayDest);

add_block(captureSource, captureDest, ...
    'Position', [1660 120 1800 205]);
add_block(displaySource, displayDest, ...
    'Position', [1870 125 1990 195]);

% Retain the exact validated OV7251 acquisition cadence. The passive camera
% was proven to deliver live frames at this 30 Hz setting; changing the
% capture block itself to 10 Hz produced a host window that held one frame.
% Python may still infer at 10 Hz without altering camera acquisition.
set_param(captureDest, ...
    'source', 'device', ...
    'device', '2', ...
    'resolution', '640 x 480', ...
    'width', '640', ...
    'height', '480', ...
    'output_format', 'MATLAB greyscale (HxW)', ...
    'data_type', 'uint8', ...
    'sample_time', 'qc_get_step_size * ceil(1/30 / qc_get_step_size)');

add_line(targetModel, ...
    'Downward Grayscale Camera Capture/1', ...
    'Optical Flow Camera feed/1', ...
    'autorouting', 'on');

% Output 2 is the camera's new-frame indicator. It is not required by the
% compressed display, but terminate it explicitly to keep the generated
% model diagnostic-free.
newFrameTerminator = [targetModel '/Unused Camera New Frame Flag'];
deleteIfPresent(newFrameTerminator);
add_block('simulink/Sinks/Terminator', newFrameTerminator, ...
    'Position', [1830 220 1850 240]);
add_line(targetModel, ...
    'Downward Grayscale Camera Capture/2', ...
    'Unused Camera New Frame Flag/1', ...
    'autorouting', 'on');

% Explicitly preserve the flight-tested deployment and communication URI.
set_param(targetModel, 'ExtModeMexArgs', ...
    ' ''-w -d /tmp -uri %u -URI_Host tcpip://192.168.2.11:18001'',''tcpip://192.168.2.15:17001?retries=10''');

close_system(sourceCamera, 0);
set_param(targetModel, 'SimulationCommand', 'update');
fprintf('Updated model: %s\n', targetModel);

if doBuild
    slbuild(targetModel);
end

save_system(targetModel, targetFile);
fprintf('\nIntegrated-camera Commander created in:\n%s\n', targetFile);
fprintf('Use it with Mission_Server_QDrone_YOLO_Interrupt.\n');
fprintf('Do NOT run QDrone_Downward_Camera_Passive at the same time.\n');
end

function assertBlockExists(path)
try
    get_param(path, 'Handle');
catch
    error('Required source block not found: %s', path);
end
end

function deleteIfPresent(path)
handle = getSimulinkBlockHandle(path, false);
if handle ~= -1
    delete_block(path);
end
end
