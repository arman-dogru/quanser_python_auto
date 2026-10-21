function [enable, x_ref, max_x_vel, max_x_accel] = fcn(sigmoid_type, x_des, slow_lim, fast_lim, x_cur, x_vel, x_meas)

% Note: Some of the cases below are identical. However, they are separated out 
% based on the states of the state machine for ease of modification in the future. 
% For example, states 4 and 5 are identical. However, if the user wants different 
% signmoid behaviour when taking off than when landing, these cases may not be the same. 

enable = 0;
x_ref = 0;
max_x_vel = 0;
max_x_accel = 0;

switch sigmoid_type
    case 1 % This is for the IDLE, ARM, E_ABORT, E_STOP and DISABLE states of the FSM
        enable = 1;
        x_ref = x_meas;
        max_x_vel = slow_lim(1);
        max_x_accel = slow_lim(2);

    case 2 % This is for the TAKEOFF THROTTLE state of the FSM
        enable = 1;
        x_ref = x_meas;
        max_x_vel = slow_lim(1);
        max_x_accel = slow_lim(2);

    case 3 % This is for the FLIGHT state of the FSM
        enable = 1;
        x_ref = x_des;
        max_x_vel = fast_lim(1);
        max_x_accel = fast_lim(2);

    case 4 % This is for the TAKEOFF HEIGHT state of the FSM
        enable = 1;
        x_ref = x_des;
        if (abs(x_cur - x_des) <= 0.02)
            max_x_vel = fast_lim(1);
            max_x_accel = fast_lim(2);
        else    
            max_x_vel = slow_lim(1);
            max_x_accel = slow_lim(2);
        end
    case 5 % This is for the AUTOLAND state of the FSM
        enable = 1;
        x_ref = x_des;
        if (abs(x_cur - x_des) <= 0.02)
            max_x_vel = fast_lim(1);
            max_x_accel = fast_lim(2);        
        else
            max_x_vel = slow_lim(1);
            max_x_accel = slow_lim(2);
        end
        
end
