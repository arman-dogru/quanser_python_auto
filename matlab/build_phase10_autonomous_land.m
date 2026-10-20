function build_phase10_autonomous_land(doBuild)
%BUILD_PHASE10_AUTONOMOUS_LAND Create a separate auto-LAND Mission Server.
% Requires the Phase 9 diagnostic model and does not change the Commander.
if nargin < 1, doBuild = false; end
projectRoot = fileparts(fileparts(mfilename('fullpath')));
modelDir = fullfile(projectRoot,'simulink');
sourceModel = 'Mission_Server_QDrone_YOLO_Landing_Diagnostics';
targetModel = 'Mission_Server_QDrone_YOLO_Autonomous_Land';
sourceFile = fullfile(modelDir,[sourceModel '.mdl']);
targetFile = fullfile(modelDir,[targetModel '.mdl']);
assert(isfile(sourceFile),'Source model missing: %s',sourceFile);
assert(~isfile(targetFile),'Refusing to overwrite existing model: %s',targetFile);
for name = {sourceModel,targetModel}
    if bdIsLoaded(name{1})
        assert(strcmp(get_param(name{1},'Dirty'),'off'), ...
            'Save open model changes before running: %s',name{1});
        close_system(name{1},0);
    end
end
load_system(sourceFile);
save_system(sourceModel,targetFile);
close_system(sourceModel,0);
load_system(targetFile);
mission = firstBlock(targetModel,'MISSION SERVER');
missionName = getfullname(mission);
gate = firstBlock(mission,'Landing Readiness Gate');
out = firstBlock(mission,'Triggers (0,1) [4]');
ph = get_param(out,'PortHandles');
oldLine = get_param(ph.Inport(1),'Line');
assert(oldLine ~= -1,'Trigger output is not connected.');
oldSource = get_param(oldLine,'SrcPortHandle');
assert(oldSource ~= -1,'Cannot identify operator trigger source.');

arbiter = add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [missionName '/Autonomous LAND Trigger Arbiter'], ...
    'Position',[1510 1080 1710 1170]);
chart = find(sfroot,'-isa','Stateflow.EMChart','Path',getfullname(arbiter));
assert(~isempty(chart),'Autonomous LAND arbiter chart missing.');
chart.Script = fileread(fullfile(projectRoot,'matlab', ...
    'phase10_autoland_arbiter_code.txt'));
ap = get_param(arbiter,'PortHandles');
gp = get_param(gate,'PortHandles');
assert(numel(ap.Inport)==2 && numel(ap.Outport)==2, ...
    'Autonomous LAND arbiter port mismatch.');

% The mission's original joystick signal remains untouched for search,
% descent and manual controls. Modify only its output sent to Commander.
add_line(missionName,oldSource,ap.Inport(1),'autorouting','on');
add_line(missionName,gp.Outport(1),ap.Inport(2),'autorouting','on');
delete_line(missionName,oldSource,ph.Inport(1));
add_line(missionName,ap.Outport(1),ph.Inport(1),'autorouting','on');
display = add_block('simulink/Sinks/Display', ...
    [missionName '/Autonomous LAND Request Display'], ...
    'Position',[1750 1090 1860 1125]);
dp = get_param(display,'PortHandles');
add_line(missionName,ap.Outport(2),dp.Inport(1),'autorouting','on');

set_param(targetModel,'SimulationCommand','update');
save_system(targetModel,targetFile);
fprintf('Autonomous LAND model created: %s\n',targetFile);
if doBuild, slbuild(targetModel); end
fprintf('Commander remains unchanged. Never run this model before dry-run validation.\n');
end

function block = firstBlock(system,fragment)
blocks = find_system(system,'FindAll','on','SearchDepth',1,'Type','Block');
blocks(blocks==get_param(system,'Handle')) = [];
match = false(size(blocks));
for k=1:numel(blocks)
    match(k)=contains(get_param(blocks(k),'Name'),fragment,'IgnoreCase',true);
end
blocks=blocks(match);
assert(numel(blocks)==1,'Expected exactly one block containing "%s".',fragment);
block=blocks(1);
end
