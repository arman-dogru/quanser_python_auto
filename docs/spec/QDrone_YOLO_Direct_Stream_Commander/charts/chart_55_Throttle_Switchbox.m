function sigmoid  = application_drone_throttle_switchbox(FSM_state)

% Application Drone: Throttle Switchbox
%
% Logic for switching between throttle commands from the controller 
%
% v2.0 - 5th october, 2018
%
% INPUTS:
% -------
% FSM_state = current state of the FSM------------------------------------------------------- (#)         [1]
% 
% OUTPUTS:
% --------
% sigmoid = an integer whose state determines how the heights are transformed---------------- (#)         [1]
% sigmoid = 1 - Command 0 throttle
% sigmoid = 2 - Sigmoid throttle up from 0 to the trim value
% sigmoid = 3 - Pass controller throttle straight through + trim value.
% sigmoid = 4 - Ignore controller throttle. Sigmoid it down to 0.
% sigmoid = 5 - Pass controller throttle straight through + trim value.


sigmoid = 0;
switch FSM_state
    case 1 % IDLE STATE
        sigmoid = 1;
    case 2 % ARM STATE
        sigmoid = 1;
    case 3 % TAKEOFF THROTTLE STATE
        sigmoid = 2;
    case 4 % TAKEOFF HEIGHT STATE
        sigmoid = 3;
    case 5 % FLIGHT STATE
        sigmoid = 3;
    case 6 % AUTOLAND STATE
        sigmoid = 5;
    case 7 % EMERGENCY STOP STATE
        sigmoid = 4;
    case 8 % EMERGENCY ABORT STATE
        sigmoid = 1;
    case 9 % DISABLE STATE
        sigmoid = 1;
end   
