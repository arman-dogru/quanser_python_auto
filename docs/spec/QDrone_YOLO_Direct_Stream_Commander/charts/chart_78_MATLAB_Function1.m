function [enable, y_ref, max_y_vel, max_y_accel] = fcn(sigmoid_type, y_des, slow_lim, fast_lim, y_cur, y_vel, y_meas)

% Note: Some of the cases below are identical. However, they are separated out 
% based on the states of the state machine for ease of modification in the future. 
% For example, states 4 and 5 are identical. However, if the user wants different 
% signmoid behaviour when taking off than when landing, these cases may not be the same. 

enable = 0;
y_ref = 0;
max_y_vel = 0;
max_y_accel = 0;

switch sigmoid_type
    case 1 % This is for the IDLE, ARM, E_ABORT, E_STOP and DISABLE states of the FSM
        enable = 1;
        y_ref = y_meas;
        max_y_vel = slow_lim(1);
        max_y_accel = slow_lim(2);

    case 2 % This is for the TAKEOFF THROTTLE state of the FSM
        enable = 1;
        y_ref = y_meas;
        max_y_vel = slow_lim(1);
        max_y_accel = slow_lim(2);

    case 3 % This is for the FLIGHT state of the FSM
        enable = 1;
        y_ref = y_des;
        max_y_vel = fast_lim(1);
        max_y_accel = fast_lim(2);

    case 4 % This is for the TAKEOFF HEIGHT state of the FSM
        enable = 1;
        y_ref = y_des;
        if (abs(y_cur - y_des) <= 0.02)
            max_y_vel = fast_lim(1);
            max_y_accel = fast_lim(2);
        else    
            max_y_vel = slow_lim(1);
            max_y_accel = slow_lim(2);
        end
    case 5 % This is for the AUTOLAND state of the FSM
        enable = 1;
        y_ref = y_des;
        if (abs(y_cur - y_des) <= 0.02)
            max_y_vel = fast_lim(1);
            max_y_accel = fast_lim(2);        
        else
            max_y_vel = slow_lim(1);
            max_y_accel = slow_lim(2);
        end
        
end
