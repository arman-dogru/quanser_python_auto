function test_phase9_landing_gate()
%TEST_PHASE9_LANDING_GATE Software-only checks for the diagnostic gate.
projectRoot=fileparts(fileparts(mfilename('fullpath')));
script=fileread(fullfile(projectRoot,'matlab','phase9_landing_gate_code.txt'));
assert(contains(script,'= fcn('),'Unexpected gate function signature.');
script=strrep(script,'= fcn(','= phase9_gate_under_test(');
testDir=tempname;mkdir(testDir);
cleanup=onCleanup(@() cleanTestDir(testDir));
fid=fopen(fullfile(testDir,'phase9_gate_under_test.m'),'w');
assert(fid~=-1,'Could not create temporary test function.');
fwrite(fid,script,'char');fclose(fid);
addpath(testDir,'-begin');
clear phase9_gate_under_test;
measured=[0;0;0.55;0];desired=measured;

% No readiness before low hover, even with repeated fresh observations.
for k=1:12
 [ready,~,reason]=phase9_gate_under_test(4,0,1,1,true,1,measured,desired,k*0.20);
 assert(~ready && reason==1,'Gate armed before stage 5.');
end

% Stage 5 needs ten new observations, not ten model ticks.
for k=1:9
 [ready,count,~]=phase9_gate_under_test(5,0,1,1,true,1,measured,desired,3+k*0.20);
 assert(~ready && count==k,'Gate counted or armed too early.');
end
[ready,count,reason]=phase9_gate_under_test(5,0,1,1,true,1,measured,desired,5.0);
assert(ready && count==10 && reason==0,'Ten fresh observations did not arm gate.');
fprintf('PASS: landing readiness requires stage 5 and ten fresh observations.\n');

% Repeated model ticks are not fresh observations; stale vision disarms.
[ready,count,~]=phase9_gate_under_test(5,0,1,1,false,1,measured,desired,5.3);
assert(ready && count==10,'A brief gap unexpectedly cleared readiness.');
[ready,count,reason]=phase9_gate_under_test(5,0,1,1,false,1,measured,desired,5.6);
assert(~ready && count==0 && reason==6,'Stale vision did not disarm gate.');
fprintf('PASS: stale vision disarms landing readiness.\n');

% A latched descent abort has priority over new vision.
[ready,count,reason]=phase9_gate_under_test(5,1,1,1,true,1,measured,desired,5.8);
assert(~ready && count==0 && reason==2,'Descent abort did not inhibit gate.');
fprintf('PASS: descent abort inhibits landing readiness.\n');

% Reject tracking loss, off-center pose, and incorrect height.
[ready,~,reason]=phase9_gate_under_test(5,0,1,1,true,0,measured,desired,6.0);
assert(~ready && reason==4,'Tracking loss did not inhibit gate.');
offCenter=measured;offCenter(1)=0.09;
[ready,~,reason]=phase9_gate_under_test(5,0,1,1,true,1,offCenter,desired,6.2);
assert(~ready && reason==5,'Position error did not inhibit gate.');
tooHigh=measured;tooHigh(3)=0.60;
[ready,~,reason]=phase9_gate_under_test(5,0,1,1,true,1,tooHigh,desired,6.4);
assert(~ready && reason==5,'Height error did not inhibit gate.');
fprintf('PASS: tracking, alignment, and height checks inhibit readiness.\n');
fprintf('Phase 9 diagnostic-gate tests passed. No LAND signal was generated.\n');
end

function cleanTestDir(folder)
rmpath(folder);
clear phase9_gate_under_test;
if isfolder(folder),rmdir(folder,'s');end
end
