function test_phase9_abort_diagnostics()
%TEST_PHASE9_ABORT_DIAGNOSTICS Test the passive abort-cause latch offline.
projectRoot=fileparts(fileparts(mfilename('fullpath')));
script=fileread(fullfile(projectRoot,'matlab','phase9_abort_diagnostic_code.txt'));
assert(contains(script,'= fcn('),'Unexpected diagnostic function signature.');
script=strrep(script,'= fcn(','= phase9_abort_under_test(');
testDir=tempname;mkdir(testDir);
cleanup=onCleanup(@() cleanTestDir(testDir));
fid=fopen(fullfile(testDir,'phase9_abort_under_test.m'),'w');
assert(fid~=-1,'Could not create temporary test function.');
fwrite(fid,script,'char');fclose(fid);
addpath(testDir,'-begin');
clear phase9_abort_under_test;
pose=[0;0;0.70;0];

[bits,age,t]=phase9_abort_under_test(1,0,1,1,pose,0.1,10);
assert(bits==0 && age==0 && t==0,'Diagnostic did not start clear.');
[bits,age,t]=phase9_abort_under_test(1,1,0,1,pose,0.58,10.58);
assert(bits==1 && abs(age-0.58)<1e-9 && abs(t-10.58)<1e-9, ...
 'Vision-loss cause or age was not captured.');
[bits,age,t]=phase9_abort_under_test(1,1,1,0,pose,0.02,10.60);
assert(bits==1 && abs(age-0.58)<1e-9 && abs(t-10.58)<1e-9, ...
 'Later inputs overwrote the first abort cause.');
fprintf('PASS: first vision-loss cause and packet age are latched.\n');

[bits,~,~]=phase9_abort_under_test(0,0,0,1,pose,0,11);
assert(bits==0,'New cycle did not reset the diagnostic latch.');
[bits,~,~]=phase9_abort_under_test(1,1,1,0,pose,0.1,12);
assert(bits==2,'Tracking-loss bit was not captured.');
fprintf('PASS: tracking loss is identified after a reset.\n');

[bits,~,~]=phase9_abort_under_test(0,0,0,1,pose,0,13);
badPose=pose;badPose(1)=NaN;badPose(3)=0.40;
[bits,~,~]=phase9_abort_under_test(1,1,0,0,badPose,0.8,14);
assert(bits==15,'Simultaneous abort conditions were not preserved.');
fprintf('PASS: simultaneous abort conditions retain every cause bit.\n');
fprintf('Abort-diagnostic software tests passed. No target was run.\n');
end

function cleanTestDir(folder)
rmpath(folder);
clear phase9_abort_under_test;
if isfolder(folder),rmdir(folder,'s');end
end
