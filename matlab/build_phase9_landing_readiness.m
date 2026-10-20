function build_phase9_landing_readiness(doBuild)
%BUILD_PHASE9_LANDING_READINESS Clone Phase 8 and add a diagnostic gate.
% This version has NO automatic LAND output or Commander-trigger wiring.
if nargin<1,doBuild=false;end
projectRoot=fileparts(fileparts(mfilename('fullpath')));
modelDir=fullfile(projectRoot,'simulink');
sourceFile=fullfile(modelDir,'Mission_Server_QDrone_YOLO_Guarded_Descent.mdl');
targetFile=fullfile(modelDir,'Mission_Server_QDrone_YOLO_Landing_Readiness.mdl');
sourceModel='Mission_Server_QDrone_YOLO_Guarded_Descent';
targetModel='Mission_Server_QDrone_YOLO_Landing_Readiness';
assert(isfile(sourceFile),'Phase 8 model missing: %s',sourceFile);
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
gatePath=[missionName '/Landing Readiness Gate'];
gate=add_block('simulink/User-Defined Functions/MATLAB Function',gatePath, ...
 'Position',[1510 920 1700 1010]);
chart=find(sfroot,'-isa','Stateflow.EMChart','Path',getfullname(gate));
assert(~isempty(chart),'Landing Readiness Gate MATLAB Function missing.');
chart.Script=fileread(fullfile(projectRoot,'matlab','phase9_landing_gate_code.txt'));

safeAddLine(mission,grid,8,gate,1);       % descentStage
safeAddLine(mission,grid,9,gate,2);       % descentAbort
safeAddLine(mission,confirm,1,gate,3);    % confirmedPad
safeAddLine(mission,grid,6,gate,4);       % centered
safeAddLine(mission,confirm,6,gate,5);    % newPadSample
safeAddLine(mission,conditioning,2,gate,6); % isTracking
safeAddLine(mission,conditioning,1,gate,7); % measuredPose
safeAddLine(mission,grid,1,gate,8);       % desiredPose
safeAddLine(mission,clock,1,gate,9);      % clockTime

addDisplay(missionName,'Landing Ready Display',[1740 925 1850 955]);
addDisplay(missionName,'Landing Fresh Count Display',[1740 965 1850 995]);
addDisplay(missionName,'Landing Gate Reason Display',[1740 1005 1850 1035]);
safeAddLine(mission,gate,1,[missionName '/Landing Ready Display'],1);
safeAddLine(mission,gate,2,[missionName '/Landing Fresh Count Display'],1);
safeAddLine(mission,gate,3,[missionName '/Landing Gate Reason Display'],1);
set_param(targetModel,'SimulationCommand','update');
save_system(targetModel,targetFile);
fprintf('Updated diagnostic model: %s\n',targetModel);
if doBuild,slbuild(targetModel);end
fprintf('Landing-readiness model: %s\n',targetFile);
fprintf('No automatic LAND command or Commander trigger was added.\n');
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
