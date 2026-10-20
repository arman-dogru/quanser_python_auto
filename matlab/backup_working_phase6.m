function backup_working_phase6()
%BACKUP_WORKING_PHASE6 Preserve the flight-tested 2026-09-15 YOLO-hover pair.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
backupDir = fullfile(projectRoot, 'backups', ...
    'working_yolo_hover_2026-09-15');
if ~isfolder(backupDir), mkdir(backupDir); end

files = { ...
    fullfile('simulink','Mission_Server_QDrone_YOLO_Interrupt.mdl'), ...
    fullfile('simulink','QDrone_YOLO_Direct_Stream_Commander.mdl'), ...
    fullfile('python','yolo_pad_direct_stream.py'), ...
    fullfile('matlab','config_autonomous.m'), ...
    fullfile('matlab','build_phase6_direct_camera_stream.m')};

for k = 1:numel(files)
    source = fullfile(projectRoot, files{k});
    assert(isfile(source), 'Working Phase 6 file is missing: %s', source);
    [~,name,ext] = fileparts(source);
    destination = fullfile(backupDir, [name ext]);
    assert(copyfile(source, destination, 'f') == 1, ...
        'Could not back up: %s', source);
    fprintf('Backed up: %s\n', destination);
end

manifest = fullfile(backupDir, 'README.txt');
fid = fopen(manifest, 'w');
assert(fid ~= -1, 'Could not create backup manifest: %s', manifest);
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, ['Flight-tested 2026-09-15 baseline.\n' ...
    'Verified behavior: full grid search, direct grayscale TCP camera, ' ...
    'YOLO confirmation, grid interruption, stable hover.\n' ...
    'Do not edit these files in place.\n']);
fprintf('\nProtected Phase 6 backup created at:\n%s\n', backupDir);
end
