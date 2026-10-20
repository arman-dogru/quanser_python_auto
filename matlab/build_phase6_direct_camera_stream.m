function build_phase6_direct_camera_stream(doBuild)
%BUILD_PHASE6_DIRECT_CAMERA_STREAM Add screenless camera TCP to Commander.
% Creates a new model. Verified Commander and backup models are untouched.

if nargin < 1, doBuild = false; end

projectRoot = fileparts(fileparts(mfilename('fullpath')));
modelDir = fullfile(projectRoot, 'simulink');
sourceCommanderFile = fullfile(modelDir, 'QDrone_YOLO_Interrupt_Commander.mdl');
sourceCameraFile = fullfile(modelDir, 'QDrone_Downward_Camera_Passive.slx');
targetFile = fullfile(modelDir, 'QDrone_YOLO_Direct_Stream_Commander.mdl');
sourceCommander = 'QDrone_YOLO_Interrupt_Commander';
sourceCamera = 'QDrone_Downward_Camera_Passive';
targetModel = 'QDrone_YOLO_Direct_Stream_Commander';

assert(isfile(sourceCommanderFile), 'Missing YOLO Commander: %s', sourceCommanderFile);
assert(isfile(sourceCameraFile), 'Missing camera source: %s', sourceCameraFile);

for name = {sourceCommander, sourceCamera, targetModel}
    if bdIsLoaded(name{1}), close_system(name{1}, 0); end
end

load_system(sourceCommanderFile);
save_system(sourceCommander, targetFile);
close_system(sourceCommander, 0);
load_system(targetFile);
load_system(sourceCameraFile);

captureSource = [sourceCamera '/Video Capture'];
assert(getSimulinkBlockHandle(captureSource, false) ~= -1, ...
    'Validated Video Capture block is missing from camera source.');

capture = [targetModel '/Downward Grayscale Camera Capture'];
rate = [targetModel '/Camera Network Rate 5 Hz'];
enable = [targetModel '/Enable Camera Stream'];
stream = [targetModel '/Direct Grayscale TCP Stream'];
terms = [targetModel '/Camera Stream Diagnostics'];
newFrameTerm = [targetModel '/Unused Camera New Frame Flag'];
for block = {capture, rate, enable, stream, terms, newFrameTerm}
    deleteIfPresent(block{1});
end

add_block(captureSource, capture, 'Position', [1590 105 1730 190]);
set_param(capture, ...
    'source', 'device', 'device', '2', ...
    'resolution', '640 x 480', 'width', '640', 'height', '480', ...
    'output_format', 'MATLAB greyscale (HxW)', 'data_type', 'uint8', ...
    'sample_time', 'qc_get_step_size * ceil(1/30 / qc_get_step_size)');

add_block('simulink/Signal Attributes/Rate Transition', rate, ...
    'Position', [1770 110 1870 160], ...
    'OutPortSampleTime', '0.2', ...
    'Integrity', 'off', ...
    'Deterministic', 'off');
add_block('simulink/Sources/Constant', enable, ...
    'Position', [1800 185 1840 215], 'Value', 'true', ...
    'OutDataTypeStr', 'boolean', 'SampleTime', '0.2');

add_block('quarc_library/Communications/Basic/Stream Client', stream, ...
    'Position', [1910 100 2080 195]);
set_param(stream, ...
    'uri_source', 'Specify via dialog (do not evaluate)', ...
    'uri', 'tcpip://192.168.2.11:18910', ...
    'send_options', 'Send most recent data', ...
    'receive_options', 'Do not receive data', ...
    'sample_time', '0.2', ...
    'send_buffer_size', '1048576', ...
    'optimization', 'Minimum latency', ...
    'implementation', 'Use blocking I/O in a separate thread', ...
    'send_fifo_size', '1');

add_block('simulink/Sinks/Terminator', newFrameTerm, ...
    'Position', [1775 235 1795 255]);
add_block('simulink/Signal Routing/Bus Creator', terms, ...
    'Position', [2130 110 2135 185], 'Inputs', '3');
diagTerm = [targetModel '/Unused Camera Stream Diagnostics'];
deleteIfPresent(diagTerm);
add_block('simulink/Sinks/Terminator', diagTerm, ...
    'Position', [2175 135 2195 155]);

add_line(targetModel, 'Downward Grayscale Camera Capture/1', ...
    'Camera Network Rate 5 Hz/1', 'autorouting', 'on');
add_line(targetModel, 'Camera Network Rate 5 Hz/1', ...
    'Direct Grayscale TCP Stream/1', 'autorouting', 'on');
add_line(targetModel, 'Enable Camera Stream/1', ...
    'Direct Grayscale TCP Stream/2', 'autorouting', 'on');
add_line(targetModel, 'Downward Grayscale Camera Capture/2', ...
    'Unused Camera New Frame Flag/1', 'autorouting', 'on');

ports = get_param(stream, 'PortHandles');
assert(numel(ports.Outport) == 3, ...
    'Expected Stream Client outputs state, err and sent.');
for k = 1:3
    add_line(targetModel, ports.Outport(k), ...
        get_param(terms, 'PortHandles').Inport(k), 'autorouting', 'on');
end
add_line(targetModel, 'Camera Stream Diagnostics/1', ...
    'Unused Camera Stream Diagnostics/1', 'autorouting', 'on');

set_param(targetModel, 'ExtModeMexArgs', ...
    ' ''-w -d /tmp -uri %u -URI_Host tcpip://192.168.2.11:18001'',''tcpip://192.168.2.15:17001?retries=10''');

close_system(sourceCamera, 0);
set_param(targetModel, 'SimulationCommand', 'update');
fprintf('Updated model: %s\n', targetModel);
if doBuild, slbuild(targetModel); end
save_system(targetModel, targetFile);
fprintf('\nDirect-camera Commander created in:\n%s\n', targetFile);
fprintf('Start ground YOLO before this Commander. No camera display is used.\n');
end

function deleteIfPresent(path)
handle = getSimulinkBlockHandle(path, false);
if handle ~= -1, delete_block(path); end
end
