function build_phase9_abort_diagnostics(doBuild)
%BUILD_PHASE9_ABORT_DIAGNOSTICS Add read-only diagnostics to a model copy.
% Does not change flight, centering, descent, Commander, or LAND behavior.
if nargin<1,doBuild=false;end
projectRoot=fileparts(fileparts(mfilename('fullpath')));
modelDir=fullfile(projectRoot,'simulink');
sourceFile=fullfile(modelDir,'Mission_Server_QDrone_YOLO_Landing_Readiness.mdl');
targetFile=fullfile(modelDir,'Mission_Server_QDrone_YOLO_Landing_Diagnostics.mdl');
sourceModel='Mission_Server_QDrone_YOLO_Landing_Readiness';
targetModel='Mission_Server_QDrone_YOLO_Landing_Diagnostics';
assert(isfile(sourceFile),'Landing-readiness model missing: %s',sourceFile);
for name={sourceModel,targetModel}
 if bdIsLoaded(name{1})
  assert(strcmp(get_param(name{1},'Dirty'),'off'), ...
   'Save changes to the open model before running this builder: %s',name{1});
  close_system(name{1},0);
 end
end
load_system(sourceFile);
save_system(sourceModel,targetFile);
close_system(sourceModel,0);
load_system(targetFile);
mission=firstBlock(targetModel,'MISSION SERVER');
missionName=getfullname(mission);
grid=firstBlock(mission,'Grid Search Manager');
confirm=firstBlock(mission,'YOLO Confirmation and Hover Hold');
conditioning=firstBlock(mission,'Measurement Conditioning');
clock=firstBlock(mission,'YOLO Freshness Clock');

% Add one observation-only output to the existing confirmation block.
chart=find(sfroot,'-isa','Stateflow.EMChart','Path',getfullname(confirm));
assert(~isempty(chart),'YOLO confirmation MATLAB Function not found.');
script=chart.Script;
oldSignature='newPadSample] = fcn(';
assert(contains(script,oldSignature),'Unexpected YOLO confirmation signature.');
script=strrep(script,oldSignature,'newPadSample,visionAge] = fcn(');
assert(~isempty(regexp(script,'\nend\s*$','once')), ...
 'Could not locate the end of the YOLO confirmation function.');
script=regexprep(script,'\nend\s*$',sprintf([ ...
 '\nif lastFresh<0,visionAge=Inf;else,visionAge=max(0,clockTime-lastFresh);end\nend']));
chart.Script=script;

diagnosticPath=[missionName '/Abort Diagnostic'];
diagnostic=add_block('simulink/User-Defined Functions/MATLAB Function', ...
 diagnosticPath,'Position',[1510 1100 1710 1190]);
diagnosticChart=find(sfroot,'-isa','Stateflow.EMChart','Path',getfullname(diagnostic));
assert(~isempty(diagnosticChart),'Abort Diagnostic MATLAB Function not found.');
diagnosticChart.Script=fileread(fullfile(projectRoot,'matlab', ...
 'phase9_abort_diagnostic_code.txt'));

safeAddLine(mission,grid,8,diagnostic,1);         % descentStage
safeAddLine(mission,grid,9,diagnostic,2);         % descentAbort
safeAddLine(mission,confirm,1,diagnostic,3);      % confirmedPad
safeAddLine(mission,conditioning,2,diagnostic,4);% isTracking
safeAddLine(mission,conditioning,1,diagnostic,5);% measuredPose
safeAddLine(mission,confirm,7,diagnostic,6);      % visionAge
safeAddLine(mission,clock,1,diagnostic,7);        % clockTime

addDisplay(missionName,'Current Vision Age Display',[1750 1070 1860 1100]);
addDisplay(missionName,'Abort Cause Bits Display',[1750 1115 1860 1145]);
addDisplay(missionName,'Vision Age At Abort Display',[1750 1160 1860 1190]);
addDisplay(missionName,'Abort Time Display',[1750 1205 1860 1235]);
safeAddLine(mission,confirm,7,[missionName '/Current Vision Age Display'],1);
safeAddLine(mission,diagnostic,1,[missionName '/Abort Cause Bits Display'],1);
safeAddLine(mission,diagnostic,2,[missionName '/Vision Age At Abort Display'],1);
safeAddLine(mission,diagnostic,3,[missionName '/Abort Time Display'],1);
set_param(targetModel,'SimulationCommand','update');
save_system(targetModel,targetFile);
fprintf('Updated diagnostic model: %s\n',targetModel);
if doBuild,slbuild(targetModel);end
fprintf('Abort-diagnostic model: %s\n',targetFile);
fprintf('No flight-command, timeout, or LAND logic was changed.\n');
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
