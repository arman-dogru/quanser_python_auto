function [desiredPose,waypointIndex,gridComplete,visitedCount,centeringActive,centered,pixelError,descentStage,descentAbort] = fcn(measuredPose,homePose,triggers,confirmedPad,holdPose,padU,padV,newPadSample,isTracking)
%#codegen
% Stage 0: search; 1: center at 0.70; 2: descend to 0.60;
% 3: center at 0.60; 4: descend to 0.55; 5: low hover.
persistent stage dwell takeoffWasLow reverseOrder complete visited sessionActive centerMode centerPose centerOrigin centerGood descentMode abortLatched
if isempty(stage)
 stage=uint8(0);dwell=uint16(0);takeoffWasLow=true;reverseOrder=false;
 complete=false;visited=uint8(0);sessionActive=false;centerMode=false;
 centerPose=zeros(4,1);centerOrigin=zeros(2,1);centerGood=uint8(0);
 descentMode=uint8(0);abortLatched=false;
end
takeoff=triggers(2)>0.5;
wps=[-1.2 -1 0.7;-0.6 -1 0.7;0 -1 0.7;0.6 -1 0.7;1.2 -1 0.7;1.2 -0.333333 0.7;0.6 -0.333333 0.7;0 -0.333333 0.7;-0.6 -0.333333 0.7;-1.2 -0.333333 0.7;-1.2 0.333333 0.7;-0.6 0.333333 0.7;0 0.333333 0.7;0.6 0.333333 0.7;1.2 0.333333 0.7;1.2 1 0.7;0.6 1 0.7;0 1 0.7;-0.6 1 0.7;-1.2 1 0.7];
baseMap=[0.00163 0.00019;7e-05 -0.00166];
cameraOffset=[0.11;0];
imageCenter=[320;240];
pixelError=zeros(2,1);
validPose=all(isfinite(measuredPose)) && all(isfinite(homePose));
if ~takeoff
 stage=uint8(0);dwell=uint16(0);takeoffWasLow=true;reverseOrder=false;
 sessionActive=false;centerMode=false;centerGood=uint8(0);
 descentMode=uint8(0);abortLatched=false;
elseif ~sessionActive
 complete=false;visited=uint8(0);sessionActive=true;
end
if takeoff && confirmedPad && isTracking>0.5 && validPose && ~centerMode
 centerMode=true;centerPose=holdPose;centerPose(3)=0.70;
 centerOrigin=holdPose(1:2);centerGood=uint8(0);descentMode=uint8(1);
end
desiredPose=homePose;desiredPose(3)=0.70;
if centerMode
 if isTracking<=0.5 || ~validPose || ~confirmedPad || measuredPose(3)<0.45
  abortLatched=true;centerGood=uint8(0);
  if validPose && isTracking>0.5
   centerPose(3)=min(max(measuredPose(3),0.55),0.70);
  end
 end
 if ~abortLatched
  h=min(max(measuredPose(3),0.50),0.70);
  map=baseMap*(h/0.70);
  target=imageCenter-map\cameraOffset;
  pixelError=[padU-target(1);padV-target(2)];
  if newPadSample
   if norm(pixelError)<=20
    if centerGood<uint8(10),centerGood=centerGood+uint8(1);end
   else
    centerGood=uint8(0);
    bodyStep=map*pixelError;
    a=measuredPose(4)-0.015;
    R=[cos(a) -sin(a);sin(a) cos(a)];
    step=R*bodyStep;n=norm(step);
    if n>0.005,step=step*(0.005/n);end
    candidate=centerPose(1:2)+step;
    offset=candidate-centerOrigin;on=norm(offset);
    if on>0.5,candidate=centerOrigin+offset*(0.5/on);end
    xLim=[-1.2 1.2];yLim=[-1 1];
    centerPose(1)=min(max(candidate(1),xLim(1)),xLim(2));
    centerPose(2)=min(max(candidate(2),yLim(1)),yLim(2));
   end
   xyError=norm(measuredPose(1:2)-centerPose(1:2));
   if centerGood>=uint8(10) && xyError<=0.12
    if descentMode==uint8(1) && abs(measuredPose(3)-0.70)<=0.05
     descentMode=uint8(2);centerPose(3)=0.60;centerGood=uint8(0);
    elseif descentMode==uint8(2) && abs(measuredPose(3)-0.60)<=0.05
     descentMode=uint8(3);centerGood=uint8(0);
    elseif descentMode==uint8(3) && abs(measuredPose(3)-0.60)<=0.05
     descentMode=uint8(4);centerPose(3)=0.55;centerGood=uint8(0);
    elseif descentMode==uint8(4) && abs(measuredPose(3)-0.55)<=0.05
     descentMode=uint8(5);centerGood=uint8(0);
    end
   end
  end
 end
 desiredPose=centerPose;
elseif takeoff && takeoffWasLow
 eHome=measuredPose(1:3)-desiredPose(1:3);
 if norm(eHome)<=0.12
  dwell=dwell+uint16(1);
  if dwell>=uint16(100)
   dFirst=measuredPose(1:2)-wps(1,1:2);
   dLast=measuredPose(1:2)-wps(20,1:2);
   reverseOrder=sum(dLast.*dLast)<sum(dFirst.*dFirst);
   stage=uint8(1);dwell=uint16(0);takeoffWasLow=false;
  end
 else,dwell=uint16(0);end
elseif takeoff && stage>=1
 idx=double(stage);if reverseOrder,idx=21-idx;end
 desiredPose(1:3)=wps(idx,:);
 e=measuredPose(1:3)-desiredPose(1:3);
 if norm(e)<=0.12
  dwell=dwell+uint16(1);
  if dwell>=uint16(100)
   visited=stage;
   if stage<uint8(20),stage=stage+uint8(1);else,complete=true;end
   dwell=uint16(0);
  end
 else,dwell=uint16(0);end
end
waypointIndex=0;
if stage>=uint8(1)
 waypointIndex=double(stage);if reverseOrder,waypointIndex=21-waypointIndex;end
end
gridComplete=complete;visitedCount=double(visited);
centeringActive=centerMode;centered=centerGood>=uint8(10) && descentMode==uint8(5);
descentStage=double(descentMode);descentAbort=abortLatched;
end
