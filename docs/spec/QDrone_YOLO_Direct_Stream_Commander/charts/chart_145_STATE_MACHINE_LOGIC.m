function [E_Abort, isArm, pt_ARM_to_TAKEOFF_THROTTLE, pt_TAKE_OFF_HEIGHT_TO_AUTOLAND, pt_FLIGHT_TO_AUTOLAND, pt_TAKEOFF_THROTTLE_TO_TAKEOFF_HEIGHT, pt_TAKE_OFF_HEIGHT_TO_FLIGHT, pt_AUTOLAND_TO_DISABLE, pt_EMERGENCY_STOP_TO_DISABLE, pt_EMERGENCY_ABORT_TO_DISABLE, Error, stop_model]...
    = SM_logic(current_state, initializing, Tracking_Issue, Communication_Issue, low_battery, trigger_1, trigger_2, E_abort, joystick_issue,...
    net_throttle_at_trim, net_throttle_close_to_zero, ctrl_throttle_close_to_zero, take_off_successful, flying_too_low, flying_too_high, close_to_ground, sensor_failed)

% Programmatic Logic Block
%
% Logic for the FSM is generated here based on a variety of signals.
% Mostly, logic to programmatically trigger the transition to either a 
% natural next state, or an Error state, is generated here.
%
% Last modified: March, 2020
%
% INPUTS:
% -------
% current_state = previous state of the FSM-------------------------------------------------- (#)         [1]
% initializing = signal representing that the system is initializing (first 5s + buffer)- - - (bool)      [1]
% comm = communication signal representing MAVLINK connection-------------------------------- (bool)      [1]
% peripheral = signal representing joystick and optitrack connection- - - - - - - - - - - - - (bool)      [2]
% low_battery = signal representing low battery---------------------------------------------- (bool)      [1]
% trigger_1 = trigger representing ARM/DISARM - - - - - - - - - - - - - - - - - - - - - - - - (bool)      [1]
% trigger_2 = trigger representing TAKEOFF/AUTOLAND------------------------------------------ (bool)      [1]
% motors_not_spinning = signal representing the motors not spinning - - - - - - - - - - - - - (bool)      [1]
% net_throttle_at_trim = trim throttle for the application drone ---------------------------- (bool)      [1]
% net_throttle_close_to_zero = net throttle is close to zero in emergency stop mode - - - - - (bool)      [1]
% ctrl_throttle_close_to_zero = ctrl throttle is close to zero in emergency stop mode - - - - (bool)      [1]
% take_off_successful = signal representing whether drone has taken-off successfully -------- (bool)      [1]
% flying_too_low = signal representing whether the drone's height is below threshold and cmd- (bool)      [1]
% flying_too_high = signal representing whether the drone's height is above threshold-------- (bool)      [1]
% close_to_ground = signal representing whether the drone is close to the ground to autoland- (bool)      [1]
% 
% OUTPUTS:
% --------
% Error = Whether an Error is issued or not                                                             (bool)    [1]
% E_Abort = fed through for ease of understanding flow (see STATE MACHINE TRANSITION BLOCK)             (bool)    [1]
% isArm = programmatic transition state                                                                 (bool)    [1]
% pt_ARM_to_TAKEOFF_THROTTLE = programmatic transition state BEGIN_STATE_TO_FINAL_STATE                 (bool)    [1]
% pt_TAKEOFF_THROTTLE_TO_TAKEOFF_HEIGHT = programmatic transition state BEGIN_STATE_TO_FINAL_STATE      (bool)    [1]
% pt_TAKE_OFF_HEIGHT_TO_AUTOLAND = programmatic transition state BEGIN_STATE_TO_FINAL_STATE             (bool)    [1]
% pt_TAKEOFF_THROTTLE_TO_TAKEOFF_HEIGHT = programmatic transition state BEGIN_STATE_TO_FINAL_STATE      (bool)    [1]
% pt_TAKE_OFF_HEIGHT_TO_FLIGHT = programmatic transition state BEGIN_STATE_TO_FINAL_STATE               (bool)    [1]
% pt_AUTOLAND_TO_DISABLE = programmatic transition state BEGIN_STATE_TO_FINAL_STATE                     (bool)    [1]
% pt_EMERGENCY_STOP_TO_DISABLE = programmatic transition state BEGIN_STATE_TO_FINAL_STATE               (bool)    [1]
% pt_EMERGENCY_ABORT_TO_DISABLE = programmatic transition state BEGIN_STATE_TO_FINAL_STATE              (bool)    [1]
% stop_model = STOP THE MODEL AFTER                                                                     (bool)    [1]

Error = 0;
E_Abort = E_abort;
isArm = false;
pt_ARM_to_TAKEOFF_THROTTLE = false;
pt_TAKEOFF_THROTTLE_TO_TAKEOFF_HEIGHT = false;
pt_TAKE_OFF_HEIGHT_TO_AUTOLAND = false;
pt_TAKE_OFF_HEIGHT_TO_FLIGHT = false;
pt_FLIGHT_TO_AUTOLAND = false;
pt_AUTOLAND_TO_DISABLE = false;
pt_EMERGENCY_STOP_TO_DISABLE = false;
pt_EMERGENCY_ABORT_TO_DISABLE = false;
stop_model = 0;

switch current_state
    %---------------------------------------------------------------------%
    case 1 % IDLE STATE
        if (Communication_Issue)||...          % Communication issue
           (joystick_issue)||...               % joystick issue
           (Tracking_Issue)||...               % optitrack issue
           (low_battery)||...                  % low batter  
           (sensor_failed)||...                % if sensor (IMU) failure is detected on drone stabilizer    
           ((trigger_1)&&(initializing))||...  % armed while initializing
           (trigger_2)&&(initializing)||...    % tried to takeoff while initializing
           (~ctrl_throttle_close_to_zero)      % if throttle isn't at zero, don't arm
           %----------------------------------------%
            Error = 1;
        elseif trigger_1 % user arms
            isArm = true;
        end
    %---------------------------------------------------------------------%    
    case 2 % ARMED STATE
        isArm = trigger_1;
        if (Communication_Issue)||...          % Communication issue
           (joystick_issue)||...               % joystick issue
           (Tracking_Issue)||...               % optitrack issue
           (low_battery)||...                  % low battery
           (sensor_failed)||...                % if sensor (IMU) failure is detected on drone stabilizer
           (~ctrl_throttle_close_to_zero)      % if throttle isn't at zero, don't arm
           %----------------------------------------%
            Error = 1;
        elseif trigger_1 && trigger_2
            pt_ARM_to_TAKEOFF_THROTTLE = true;
        end        
    %---------------------------------------------------------------------%
    case 3 % TAKEOFF_THROTTLE STATE
        if (Communication_Issue)||...     % Communication issue
           (joystick_issue)||...          % joystick issue
           (Tracking_Issue)||...          % optitrack issue           
           (low_battery)||...             % low battery
           (sensor_failed)||...           % if sensor (IMU) failure is detected on drone stabilizer
           (~trigger_1)||...              % user disarms
           (~trigger_2)||...              % user commands auto-land before drone starts taking off
           (~ctrl_throttle_close_to_zero) % if throttle isn't at zero, don't arm
           %----------------------------------------%
            Error = 1;
        elseif (joystick_issue)           % joystick issue
            pt_TAKE_OFF_HEIGHT_TO_AUTOLAND = true;
        elseif net_throttle_at_trim
            pt_TAKEOFF_THROTTLE_TO_TAKEOFF_HEIGHT = true;
        end
    %---------------------------------------------------------------------%
    case 4 % TAKEOFF_HEIGHT STATE
        if (Communication_Issue)||... % Communication issue
           (joystick_issue)||...      % joystick issue
           (Tracking_Issue)||...      % optitrack issue
           (sensor_failed)||...       % if sensor (IMU) failure is detected on drone stabilizer    
           (flying_too_low)||...      % current height is below safety threshold 
           (flying_too_high)||...     % flying above safety margin (w.r.t optitrack)
           (~trigger_1)               % user disarms
            %----------------------------------------%
            Error = 1;
        elseif (~trigger_2)||low_battery 
            pt_TAKE_OFF_HEIGHT_TO_AUTOLAND = true;
        elseif take_off_successful
            pt_TAKE_OFF_HEIGHT_TO_FLIGHT = true;
        end
    %---------------------------------------------------------------------%
    case 5 % FLIGHT STATE
        if (Communication_Issue)||...   % Communication issue
           (joystick_issue)||...        % joystick issue
           (Tracking_Issue)||...        % optitrack issue
           (sensor_failed)||...         % if sensor (IMU) failure is detected on drone stabilizer
           (flying_too_low)||...        % current height is below safety threshold  
           (flying_too_high)||...       % flying above safety margin (w.r.t optitrack)
           (~trigger_1)                 % user disarms
           %----------------------------------------%
            Error = 1;
        elseif (~trigger_2)||low_battery
            pt_FLIGHT_TO_AUTOLAND = true;
        end
    %---------------------------------------------------------------------%
    case 6 % AUTOLAND STATE
        if (Communication_Issue)||...  % Communication issue
           (joystick_issue)||...       % joystick issue
           (Tracking_Issue)||...       % optitrack issue
           (sensor_failed)||...        % if sensor (IMU) failure is detected on drone stabilizer
           (flying_too_high)||...      % flying above safety margin (w.r.t optitrack)    
           (~trigger_1)||...           % user disarms
           (trigger_2)                 % user tries to takeoff while landing
           
           
           %----------------------------------------%
            Error = 1;
        elseif close_to_ground 
            pt_AUTOLAND_TO_DISABLE = true;
        end
    %---------------------------------------------------------------------%
    case 7 % EMERGENCY LAND STATE
        if net_throttle_close_to_zero 
            pt_EMERGENCY_STOP_TO_DISABLE = true;
        end
    %---------------------------------------------------------------------%
    case 8 % EMERGENCY ABORT STATE
        pt_EMERGENCY_ABORT_TO_DISABLE = true;
    %---------------------------------------------------------------------%
    case 9
        % Stop the model.
        stop_model = 1;
end
