function build_phase8_guarded_descent(doBuild)
%BUILD_PHASE8_GUARDED_DESCENT Create a separate, supervised descent model.
% Never edits the working Phase 7 model or the Commander. No LAND command.
if nargin < 1, doBuild = false; end
projectRoot = fileparts(fileparts(mfilename('fullpath')));
modelDir = fullfile(projectRoot,'simulink');
sourceFile = fullfile(modelDir,'Mission_Server_QDrone_YOLO_Centering.mdl');
targetFile = fullfile(modelDir,'Mission_Server_QDrone_YOLO_Guarded_Descent.mdl');
sourceModel = 'Mission_Server_QDrone_YOLO_Centering';
targetModel = 'Mission_Server_QDrone_YOLO_Guarded_Descent';
assert(isfile(sourceFile),'Working Phase 7 model missing: %s',sourceFile);
cfg = config_autonomous();
assert(abs(cfg.searchHeight-0.70)<1e-6,'Phase 8 requires the calibrated 0.70 m search height.');
for name = {sourceModel,targetModel}
    if bdIsLoaded(name{1}), close_system(name{1},0); end
end
load_system(sourceFile);
save_system(sourceModel,targetFile);
close_system(sourceModel,0);
load_system(targetFile);
mission = firstBlock(targetModel,'MISSION SERVER');
missionName = getfullname(mission);
grid = firstBlock(mission,'Grid Search Manager');
conditioning = firstBlock(mission,'Measurement Conditioning');
chart = find(sfroot,'-isa','Stateflow.EMChart','Path',getfullname(grid));
assert(~isempty(chart),'Grid Search Manager MATLAB Function not found.');
template = fileread(fullfile(projectRoot,'matlab','phase8_grid_manager_code.txt'));
template = strrep(template,'__WAYPOINTS__',mat2str(cfg.waypoints,6));
template = strrep(template,'__MAP__',mat2str(cfg.pixelToWorldAtSearchHeight,8));
template = strrep(template,'__OFFSET__',mat2str(cfg.cameraOffsetBody,8));
template = strrep(template,'__CENTER__',mat2str(cfg.cameraOpticalCenterPixel,8));
template = strrep(template,'__YAW__',mat2str(cfg.visionCalibrationYaw,8));
template = strrep(template,'__MAXSTEP__',mat2str(cfg.centerMaxStep,8));
template = strrep(template,'__MAXDISPLACEMENT__',mat2str(cfg.centerMaxDisplacement,8));
template = strrep(template,'__XTOL__',mat2str(cfg.xLimits,8));
template = strrep(template,'__YTOL__',mat2str(cfg.yLimits,8));
chart.Script = template;
safeAddLine(mission,conditioning,2,grid,9);
addDisplay(missionName,'Descent Stage Display',[1510 805 1620 840]);
addDisplay(missionName,'Descent Abort Display',[1510 855 1620 890]);
safeAddLine(mission,grid,8,[missionName '/Descent Stage Display'],1);
safeAddLine(mission,grid,9,[missionName '/Descent Abort Display'],1);
set_param(targetModel,'SimulationCommand','update');
save_system(targetModel,targetFile);
fprintf('Updated model: %s\n',targetModel);
if doBuild, slbuild(targetModel); end
fprintf('Phase 8 model: %s\n',targetFile);
fprintf('Use with QDrone_YOLO_Direct_Stream_Commander. No automatic LAND command.\n');
end

function block=firstBlock(system,fragment)
blocks=find_system(system,'FindAll','on','SearchDepth',1,'Type','Block');
blocks(blocks==get_param(system,'Handle'))=[];
match=false(size(blocks));
for k=1:numel(blocks)
    match(k)=contains(get_param(blocks(k),'Name'),fragment,'IgnoreCase',true);
end
blocks=blocks(match);
assert(~isempty(blocks),'Block containing "%s" not found.',fragment);
block=blocks(1);
end

function addDisplay(system,name,position)
path=[system '/' name];
h=getSimulinkBlockHandle(path,false);
if h~=-1,delete_block(path);end
add_block('simulink/Sinks/Display',path,'Position',position);
end

function safeAddLine(system,sourceBlock,sourcePort,destBlock,destPort)
sh=get_param(sourceBlock,'PortHandles');dh=get_param(destBlock,'PortHandles');
old=get_param(dh.Inport(destPort),'Line');
if old~=-1,delete_line(old);end
add_line(getfullname(system),sh.Outport(sourcePort),dh.Inport(destPort),'autorouting','on');
end
