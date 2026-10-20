function test_phase10_autoland_dry_run(projectRoot)
%TEST_PHASE10_AUTOLAND_DRY_RUN Test the actual Phase 10 arbiter without targets.
% Creates an in-memory Simulink harness and inspects the built model wiring.
if nargin < 1
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
end
modelFile = fullfile(projectRoot,'simulink', ...
    'Mission_Server_QDrone_YOLO_Autonomous_Land.mdl');
codeFile = fullfile(projectRoot,'matlab', ...
    'phase10_autoland_arbiter_code.txt');
assert(isfile(modelFile),'Phase 10 model missing: %s',modelFile);
assert(isfile(codeFile),'Arbiter code missing: %s',codeFile);

% Structural check: the real model's Commander-bound trigger output must
% come from the arbiter; the readiness gate must feed its second input.
[~,model] = fileparts(modelFile);
wasLoaded = bdIsLoaded(model);
if ~wasLoaded, load_system(modelFile); end
mission = firstBlock(model,'MISSION SERVER');
arbiter = firstBlock(mission,'Autonomous LAND Trigger Arbiter');
gate = firstBlock(mission,'Landing Readiness Gate');
triggerOut = firstBlock(mission,'Triggers (0,1) [4]');
ap = get_param(arbiter,'PortHandles');
gp = get_param(gate,'PortHandles');
op = get_param(triggerOut,'PortHandles');
assert(sourceOf(ap.Inport(2)) == gp.Outport(1), ...
    'Readiness gate is not connected to the arbiter.');
assert(sourceOf(op.Inport(1)) == ap.Outport(1), ...
    'Commander-bound trigger output bypasses the arbiter.');
assert(sourceOf(ap.Inport(1)) ~= -1, ...
    'Operator trigger input is disconnected.');
if ~wasLoaded, close_system(model,0); end
fprintf('PASS: Phase 10 trigger-path wiring is correct.\n');

% The in-memory harness evaluates the exact script used by the model.
harness = sprintf('phase10_arbiter_dryrun_%d',randi(1000000000));
new_system(harness);
cleaner = onCleanup(@()close_system(harness,0)); %#ok<NASGU>
set_param(harness,'Solver','FixedStepDiscrete', ...
    'FixedStep','0.1','StopTime','0.9');
t = (0:0.1:0.9)';
raw = [0 0 0 0; 1 1 0 1; 1 1 0 1; 1 1 0 1; ...
       1 1 0 1; 1 0 0 1; 1 1 0 1; 0 1 0 1; ...
       1 1 1 1; 1 1 1 1];
ready = [0;0;1;0;0;0;0;0;1;0];
mw = get_param(harness,'ModelWorkspace');
assignin(mw,'phase10RawSeries',timeseries(raw,t));
assignin(mw,'phase10ReadySeries',timeseries(ready,t));
rawBlock = add_block('simulink/Sources/From Workspace', ...
    [harness '/Operator Triggers'], ...
    'VariableName','phase10RawSeries','Position',[30 40 150 70]);
readyBlock = add_block('simulink/Sources/From Workspace', ...
    [harness '/Landing Ready'], ...
    'VariableName','phase10ReadySeries','Position',[30 110 150 140]);
fcnBlock = add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [harness '/Autonomous LAND Trigger Arbiter'], ...
    'Position',[220 55 420 140]);
chart = find(sfroot,'-isa','Stateflow.EMChart','Path',getfullname(fcnBlock));
assert(~isempty(chart),'Dry-run MATLAB Function chart missing.');
chart.Script = fileread(codeFile);
commandLog = add_block('simulink/Sinks/To Workspace', ...
    [harness '/Commanded Triggers'], ...
    'VariableName','phase10CommandLog','SaveFormat','Timeseries', ...
    'Position',[490 55 620 85]);
rawLog = add_block('simulink/Sinks/To Workspace', ...
    [harness '/Actual Arbiter Input'], ...
    'VariableName','phase10InputLog','SaveFormat','Timeseries', ...
    'Position',[175 5 305 35]);
latchLog = add_block('simulink/Sinks/To Workspace', ...
    [harness '/LAND Latch'], ...
    'VariableName','phase10LatchLog','SaveFormat','Timeseries', ...
    'Position',[490 110 620 140]);
connect(harness,rawBlock,1,fcnBlock,1);
connect(harness,rawBlock,1,rawLog,1);
connect(harness,readyBlock,1,fcnBlock,2);
connect(harness,fcnBlock,1,commandLog,1);
connect(harness,fcnBlock,2,latchLog,1);
result = sim(harness,'ReturnWorkspaceOutputs','on');
cmd = result.get('phase10CommandLog');
actualInput = result.get('phase10InputLog');
latch = result.get('phase10LatchLog');
commanded = vectorRows(cmd.Data,4);
received = vectorRows(actualInput.Data,4);
assert(size(commanded,1)==10 && size(commanded,2)==4, ...
    'Unexpected dry-run trigger dimensions.');
assert(size(received,1)==10,'Unexpected arbiter input dimensions.');
if ~isequal(commanded(:,[1 3 4]),received(:,[1 3 4]))
    disp('Columns: time, input[1:4], output[1:4], latch');
    disp([cmd.Time(:) received commanded double(latch.Data(:))]);
    error('Arm, emergency or other operator trigger was changed at the arbiter.');
end
assert(isequal(commanded(:,2)',[0 1 0 0 0 0 1 1 0 0]), ...
    'AUTOLAND request did not latch/reset as designed.');
assert(isequal(double(latch.Data(:))',[0 0 1 1 1 0 0 0 1 1]), ...
    'AUTOLAND latch state did not match the expected sequence.');
fprintf('PASS: no request while unready; request latches only on readiness.\n');
fprintf('PASS: manual LAND and disarm reset the latch.\n');
fprintf('PASS: arm, emergency and remaining controls pass through.\n');
fprintf('Phase 10 software-only dry run passed. No target was run.\n');
end

function rows = vectorRows(data,width)
rows = squeeze(data);
if size(rows,2)==width, return; end
if size(rows,1)==width, rows=rows'; return; end
error('Cannot interpret logged vector with width %d.',width);
end

function port = sourceOf(inputPort)
line = get_param(inputPort,'Line');
if line == -1, port = -1; else, port = get_param(line,'SrcPortHandle'); end
end

function connect(system,src,srcIndex,dst,dstIndex)
sp = get_param(src,'PortHandles'); dp = get_param(dst,'PortHandles');
add_line(system,sp.Outport(srcIndex),dp.Inport(dstIndex),'autorouting','on');
end

function block = firstBlock(system,fragment)
blocks = find_system(system,'FindAll','on','SearchDepth',1,'Type','Block');
blocks(blocks==get_param(system,'Handle')) = [];
match = false(size(blocks));
for k=1:numel(blocks)
    match(k)=contains(get_param(blocks(k),'Name'),fragment,'IgnoreCase',true);
end
blocks=blocks(match);
assert(numel(blocks)==1,'Expected one block containing "%s".',fragment);
block=blocks(1);
end
