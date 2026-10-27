function [causeBits,ageAtAbort,timeAtAbort] = fcn(descentStage,descentAbort,confirmedPad,isTracking,measuredPose,visionAge,clockTime)
%#codegen
% Latch all conditions present at the first abort sample. Diagnostic only.
% Bits: 1 pad confirmation lost, 2 tracking lost, 4 invalid pose,
%       8 below 0.45 m, 16 abort with none of these inputs observed.
persistent seenAbort savedBits savedAge savedTime
if isempty(seenAbort)
 seenAbort=false;savedBits=0;savedAge=0;savedTime=0;
end
if descentStage==0 && descentAbort<=0.5
 seenAbort=false;savedBits=0;savedAge=0;savedTime=0;
elseif descentAbort>0.5 && ~seenAbort
 bits=0;
 if confirmedPad<=0.5,bits=bits+1;end
 if isTracking<=0.5,bits=bits+2;end
 if ~all(isfinite(measuredPose)),bits=bits+4;end
 if isfinite(measuredPose(3)) && measuredPose(3)<0.45,bits=bits+8;end
 if bits==0,bits=16;end
 savedBits=bits;savedAge=visionAge;savedTime=clockTime;seenAbort=true;
end
causeBits=savedBits;ageAtAbort=savedAge;timeAtAbort=savedTime;
end
