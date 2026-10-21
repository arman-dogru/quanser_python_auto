function [enable, yaw_ref, max_yaw_vel, max_yaw_accel] = fcn(sigmoid_type, yaw_des, slow_lim, fast_lim, yaw_cur, yaw_vel)

% Note: Some of the cases below are identical. However, they are separated out 
% based on the states of the state machine for ease of modification in the future. 
% For example, states 4 and 5 are identical. However, if the user wants different 
% signmoid behaviour when taking off than when landing, these cases may not be the same. 

enable = 0;
yaw_ref = 0;
max_yaw_vel = 0;
max_yaw_accel = 0;

switch sigmoid_type
    case 1 % This is for the IDLE, ARM, E_ABORT, E_STOP and DISABLE states of the FSM
        enable = 0;
        yaw_ref = 0;
        max_yaw_vel = 0;
        max_yaw_accel = 0;
    case 2 % This is for the TAKEOFF THROTTLE state of the FSM
        enable = 1;
        yaw_ref = yaw_des;
        max_yaw_vel = slow_lim(1);
        max_yaw_accel = slow_lim(2);  
    case 3 % This is for the FLIGHT state of the FSM 
        enable = 1;
        yaw_ref = yaw_des;
        max_yaw_vel = fast_lim(1);
        max_yaw_accel = fast_lim(2);
    case 4 % This is for the TAKEOFF HEIGHT state of the FSM
        enable = 1;
        yaw_ref = yaw_des;
        if (yaw_vel < 0)||(yaw_des < yaw_cur)||(abs(yaw_cur-yaw_des)<=0.2)
            max_yaw_vel = fast_lim(1);
            max_yaw_accel = fast_lim(2);
        else    
            max_yaw_vel = slow_lim(1);
            max_yaw_accel = slow_lim(2);
        end
    case 5 % This is for the AUTOLAND state of the FSM
        enable = 1;
        yaw_ref = yaw_des;
        if (yaw_vel >= 0)
            max_yaw_vel = fast_lim(1);
            max_yaw_accel = fast_lim(2);
        else    
            max_yaw_vel = slow_lim(1);
            max_yaw_accel = slow_lim(2);
        end
end
