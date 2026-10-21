function [landingReady,freshCount,reasonCode] = fcn(descentStage,descentAbort,confirmedPad,centered,newPadSample,isTracking,measuredPose,desiredPose,clockTime)
%#codegen
% Diagnostic only. This block never changes a Commander trigger or pose.
% Reason: 0 ready, 1 stage, 2 abort, 3 vision, 4 tracking/pose,
%         5 alignment/height, 6 stale, 7 accumulating fresh observations.
persistent count lastFresh
if isempty(count)
 count=uint8(0);lastFresh=-1;
end
landingReady=false;reasonCode=uint8(7);
if descentStage~=5
 count=uint8(0);lastFresh=-1;reasonCode=uint8(1);
elseif descentAbort>0.5
 count=uint8(0);lastFresh=-1;reasonCode=uint8(2);
elseif confirmedPad<=0.5
 count=uint8(0);lastFresh=-1;reasonCode=uint8(3);
elseif isTracking<=0.5 || ~all(isfinite(measuredPose)) || ~all(isfinite(desiredPose)) || ~isfinite(clockTime)
 count=uint8(0);lastFresh=-1;reasonCode=uint8(4);
elseif centered<=0.5 || norm(measuredPose(1:2)-desiredPose(1:2))>0.08 || abs(measuredPose(3)-0.55)>0.04
 count=uint8(0);lastFresh=-1;reasonCode=uint8(5);
elseif ~newPadSample && (lastFresh<0 || clockTime-lastFresh>0.50)
 count=uint8(0);lastFresh=-1;reasonCode=uint8(6);
else
 if newPadSample
  if lastFresh<0 || clockTime-lastFresh>0.50
   count=uint8(0);
  end
  lastFresh=clockTime;
  if count<uint8(10),count=count+uint8(1);end
 end
 landingReady=count>=uint8(10) && lastFresh>=0 && clockTime-lastFresh<=0.50;
 if landingReady,reasonCode=uint8(0);end
end
freshCount=double(count);
end
