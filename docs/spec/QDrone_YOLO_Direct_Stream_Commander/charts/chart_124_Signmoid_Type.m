function sigmoid_type  = sigmoid_type_position(FSM_state)

% QDrone: Position Switchbox
%
% Switching between different sigmoid types based on 
% which state the FSM is in.
%
% v3.0 - 4th October, 2018
%
% INPUTS:
% -------
% FSM_state = current state of the FSM------------------------------------------------------- (#)         [1]
% 
% OUTPUTS:
% --------
% sigmoid_type = an integer whose state determines how the commands/measurements are transformed-- (#)         [1]

% Sigmoid_type = 1; This is for the IDLE, ARM, E_ABORT, E_STOP and DISABLE states of the FSM
% Sigmoid_type = 2; This is for the TAKEOFF THROTTLE state of the FSM
% Sigmoid_type = 3; This is for the FLIGHT state of the FSM
% Sigmoid_type = 4; This is for the TAKEOFF HEIGHT state of the FSM
% Sigmoid_type = 5; This is for the AUTOLAND state of the FSM

sigmoid_type = 0;

switch FSM_state
    case 1 % IDLE STATE
        sigmoid_type = 1;
    case 2 % ARM STATE
        sigmoid_type = 1;
    case 3 % TAKEOFF THROTTLE STATE
        sigmoid_type = 2;
    case 4 % TAKEOFF HEIGHT STATE
        sigmoid_type = 4;
    case 5 % FLIGHT STATE
        sigmoid_type = 3;
    case 6 % AUTOLAND STATE
        sigmoid_type = 5;
    case 7 % EMERGENCY STOP STATE
        sigmoid_type = 1;
    case 8 % EMERGENCY ABORT STATE
        sigmoid_type = 1;
    case 9 % DISABLE STATE
        sigmoid_type = 1;
end   
