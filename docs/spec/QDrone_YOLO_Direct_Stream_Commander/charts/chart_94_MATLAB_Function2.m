function [enable, height_ref, max_height_vel, max_height_accel] = fcn(sigmoid_type, height_des, slow_lim, fast_lim, height_cur, height_vel)

% Note: Some of the cases below are identical. However, they are separated out 
% based on the states of the state machine for ease of modification in the future. 
% For example, states 4 and 5 are identical. However, if the user wants different 
% signmoid behaviour when taking off than when landing, these cases may not be the same. 

enable = 0;
height_ref = 0;
max_height_vel = 0;
max_height_accel = 0;

switch sigmoid_type
    case 1 % This is for the IDLE, ARM, E_ABORT, E_STOP and DISABLE states of the FSM
        enable = 0;
        height_ref = 0;
        max_height_vel = 0;
        max_height_accel = 0;

    case 2 % This is for the TAKEOFF THROTTLE state of the FSM
        enable = 0;
        height_ref = 0;
        max_height_vel = 0;
        max_height_accel = 0;

    case 3 % This is for the FLIGHT state of the FSM 
        enable = 1;
        height_ref = height_des;
        max_height_vel = fast_lim(1);
        max_height_accel = fast_lim(2);

    case 4 % This is for the TAKEOFF HEIGHT state of the FSM
        enable = 1;
        height_ref = height_des;
        if (height_vel < 0)||(abs(height_cur - height_des) <= 0.1)||(height_des <= height_cur)
            max_height_vel = fast_lim(1);
            max_height_accel = fast_lim(2);
        else    
            max_height_vel = slow_lim(1);
            max_height_accel = slow_lim(2);
        end
    case 5 % This is for the AUTOLAND state of the FSM
        enable = 1;
        height_ref = 0;
        if height_vel >= 0
            max_height_vel = fast_lim(1);
            max_height_accel = fast_lim(2);        
        else
            max_height_vel = slow_lim(1);
            max_height_accel = slow_lim(2);
        end
        
end
