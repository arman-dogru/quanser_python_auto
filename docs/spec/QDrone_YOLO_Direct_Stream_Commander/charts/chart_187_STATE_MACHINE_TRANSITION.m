function Next_state = SM_transition(current_state, E_Abort, isArm, pt_ARM_to_TAKEOFF_THROTTLE, pt_TAKE_OFF_HEIGHT_TO_AUTOLAND, pt_FLIGHT_TO_AUTOLAND, pt_TAKEOFF_THROTTLE_TO_TAKEOFF_HEIGHT, pt_TAKE_OFF_HEIGHT_TO_FLIGHT, pt_AUTOLAND_TO_DISABLE, pt_EMERGENCY_STOP_TO_DISABLE, pt_EMERGENCY_ABORT_TO_DISABLE, Error)

% Programmatic Transition Block
%
% FINITE STATE MACHINE (FSM) with 9 states as described below, transitioning based on the 
% values of the isArm, transition signals, Error, and E_Abort
%
% Last modified: March, 2020
%
% INPUTS:
% ------
% current_state = Current state of the FSM        (#)     [1]
% E_Abort                                         (bool)  [1]
% isArm                                           (bool)  [1]
% pt_ARM_TO_TAKEOFF_THROTTLE                      (bool)  [1]
% pt_TAKE_OFF_HEIGHT_TO_AUTOLAND                  (bool)  [1]
% pt_FLIGHT_TO_AUTOLAND                           (bool)  [1]
% pt_TAKEOFF_THROTTLE_TO_TAKEOFF_HEIGHT           (bool)  [1]
% pt_TAKE_OFF_HEIGHT_TO_FLIGHT                    (bool)  [1]
% pt_AUTOLAND_TO_DISABLE                          (bool)  [1]
% pt_EMERGENCY_STOP_TO_DISABLE                    (bool)  [1]
% pt_EMERGENCY_ABORT_TO_DISABLE                   (bool)  [1]
% Error                                           (bool)  [1]
%
% OUTPUT:
% -------
% Next_state = Next state of the FSM              (#)     [1]

Next_state = double(0);
switch current_state
%-------------------------------------------------------------------------%
    case 1 % IDLE STATE
        if E_Abort
            Next_state = 8; % GOTO EMERGENCY ABORT STATE
        elseif Error
            Next_state = 7; % GOTO EMERGENCY STOP STATE
        elseif isArm
            Next_state = 2; % GOTO ARMED STATE
        else
            Next_state = 1; % STAY IN IDLE STATE
        end
%-------------------------------------------------------------------------%
    case 2 % ARMED STATE
        if E_Abort
            Next_state = 8; % GOTO EMERGENCY ABORT STATE
        elseif Error
            Next_state = 7; % GOTO EMERGENCY STOP STATE
        elseif ~isArm
            Next_state = 1; % GOTO IDLE STATE
        elseif pt_ARM_to_TAKEOFF_THROTTLE
            Next_state = 3; % GOTO TAKEOFF_THROTTLE STATE 
        else
            Next_state = 2; % STAY IN ARMED STATE
        end
%-------------------------------------------------------------------------%
    case 3 % TAKEOFF_THROTTLE STATE
        if E_Abort
            Next_state = 8; % GOTO EMERGENCY ABORT STATE
        elseif Error
            Next_state = 7; % GOTO EMERGENCY STOP STATE
        elseif pt_TAKEOFF_THROTTLE_TO_TAKEOFF_HEIGHT
            Next_state = 4; % GOTO TAKEOFF_HEIGHT STATE
        else
            Next_state = 3; % STAY IN TAKEOFF_THROTTLE STATE
        end
%-------------------------------------------------------------------------%
    case 4 % TAKEOFF_HEIGHT STATE
        if E_Abort
            Next_state = 8; % GOTO EMERGENCY ABORT STATE
        elseif Error
            Next_state = 7; % GOTO EMERGENCY STOP STATE
        elseif pt_TAKE_OFF_HEIGHT_TO_AUTOLAND
            Next_state = 6;
        elseif pt_TAKE_OFF_HEIGHT_TO_FLIGHT
            Next_state = 5; % GOTO FLIGHT STATE
        else
            Next_state = 4; % STAY IN TAKEOFF_HEIGHT STATE
        end
%-------------------------------------------------------------------------%
    case 5 % FLIGHT STATE
        if E_Abort
            Next_state = 8; % GOTO EMERGENCY ABORT STATE
        elseif Error
            Next_state = 7; % GOTO EMERGENCY STOP STATE
        elseif pt_FLIGHT_TO_AUTOLAND
            Next_state = 6; % GOTO AUTO_LAND STATE
        else
            Next_state = 5; % STAY IN FLIGHT STATE
        end
%-------------------------------------------------------------------------%       
    case 6 % AUTO_LAND STATE
        if E_Abort
            Next_state = 8; % GOTO EMERGENCY ABORT STATE
        elseif Error
            Next_state = 7; % GOTO EMERGENCY STOP STATE
        elseif pt_AUTOLAND_TO_DISABLE
            Next_state = 9; % GOTO DISABLE STATE
        else
            Next_state = 6; % STAY IN AUTO_LAND STATE
        end
%-------------------------------------------------------------------------%
    case 7 % EMERGENCY STOP STATE
        if E_Abort
            Next_state = 8; % GOTO EMERGENCY ABORT STATE
        elseif pt_EMERGENCY_STOP_TO_DISABLE
            Next_state = 9; % GOTO DISABLE STATE
        else
            Next_state = 7; % STAY IN EMERGENCY STOP STATE
        end
%-------------------------------------------------------------------------%
    case 8 % EMERGENCY ABORT
        if pt_EMERGENCY_ABORT_TO_DISABLE
            Next_state = 9; % GOTO DISABLE STATE
        else 
            Next_state = 8; % STAY IN EMERGENCY ABORT STATE
        end
%-------------------------------------------------------------------------%    
    case 9 % DISABLE
        Next_state = 9; % Always stay in DISABLE state if you are here    
end
