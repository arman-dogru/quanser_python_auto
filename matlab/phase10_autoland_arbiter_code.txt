function [commanderTriggers,autoLandLatched] = fcn(operatorTriggers,landingReady)
%#codegen
% Preserve arm, emergency stop, and other operator controls exactly.
% Only lower the existing Takeoff/Land signal after landing is qualified.
persistent latched
if isempty(latched)
    latched = false;
end

commanderTriggers = operatorTriggers;
armed = operatorTriggers(1) > 0.5;
takeoffSelected = operatorTriggers(2) > 0.5;

% A new arm/takeoff cycle is required before another autonomous request.
if ~armed || ~takeoffSelected
    latched = false;
elseif landingReady > 0.5
    latched = true;
end

if latched && armed && takeoffSelected
    commanderTriggers(2) = 0;
end
autoLandLatched = latched;
end
