function [confirmedPad,holdPose,confidence,padU,padV,newPadSample,visionAge] = fcn(measuredPose,vision,newVision,clockTime,triggers)
%#codegen
persistent good bad active captured lastFrame lastFresh latestU latestV
if isempty(good), good=uint8(0); bad=uint8(0); active=false; captured=zeros(4,1); lastFrame=-1; lastFresh=-1; latestU=0; latestV=0; end
takeoff=triggers(2)>0.5; newPadSample=false;
if ~takeoff, good=uint8(0); bad=uint8(0); active=false; lastFresh=clockTime; end
if takeoff && newVision && vision(5)~=lastFrame
 lastFrame=vision(5); lastFresh=clockTime; valid=vision(1)>0.5 && vision(4)>=0.600000 && vision(2)>=0 && vision(2)<=640 && vision(3)>=0 && vision(3)<=480;
 if valid, latestU=vision(2); latestV=vision(3); newPadSample=true; bad=uint8(0); if good<uint8(5),good=good+uint8(1);end; if ~active && good>=uint8(5),active=true;captured=measuredPose;end
 else, good=uint8(0); if bad<uint8(10),bad=bad+uint8(1);end; if active && bad>=uint8(10),active=false;end; end
end
if active && (lastFresh<0 || clockTime-lastFresh>0.500000),active=false;good=uint8(0);bad=uint8(0);end
confirmedPad=active;holdPose=captured;confidence=vision(4);padU=latestU;padV=latestV;
if lastFresh<0,visionAge=Inf;else,visionAge=max(0,clockTime-lastFresh);end
end
