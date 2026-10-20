function [cmd_pose, forward_cmd, right_cmd, state] = YOLOPoseCommandGenerator(measured_pose, detected, error_x, error_y)

% measured_pose = [x; y; z; yaw]
% cmd_pose      = [desired_x; desired_y; desired_z; desired_yaw]

persistent smooth_forward smooth_right centered_counter

if isempty(smooth_forward)
    smooth_forward = 0;
    smooth_right = 0;
    centered_counter = 0;
end

% Parameters
deadband_pixels = 40;
alpha = 0.25;

% Maximum normalized command
max_norm_cmd = 1.0;

% Small pose offset limit for safety
max_position_step = 0.05;   % meters

% Landing disabled for this first test
enable_descent = 0;
descent_step = 0.02;        % meters, later only

centered_required = 10;

% Image size
image_center_x = 320;
image_center_y = 240;

% Read measured pose
x = measured_pose(1);
y = measured_pose(2);
z = measured_pose(3);
yaw = measured_pose(4);

raw_forward = 0;
raw_right = 0;
state = 0;

if detected == 1

    x_centered = abs(error_x) <= deadband_pixels;
    y_centered = abs(error_y) <= deadband_pixels;

    if x_centered && y_centered
        centered_counter = centered_counter + 1;
        state = 2;
    else
        centered_counter = 0;
        state = 1;

        % Calibration-confirmed mapping:
        % error_x > 0 means too forward -> move backward
        % error_x < 0 means too backward -> move forward
        if ~x_centered
            raw_forward = -error_x / image_center_x;
        end

        % error_y > 0 means too right -> move left
        % error_y < 0 means too left -> move right
        if ~y_centered
            raw_right = -error_y / image_center_y;
        end

        raw_forward = min(max(raw_forward, -max_norm_cmd), max_norm_cmd);
        raw_right = min(max(raw_right, -max_norm_cmd), max_norm_cmd);
    end

    if centered_counter >= centered_required
        state = 3;
    end

else
    centered_counter = 0;
    state = 0;
    raw_forward = 0;
    raw_right = 0;
end

% Smooth commands
smooth_forward = alpha * raw_forward + (1 - alpha) * smooth_forward;
smooth_right = alpha * raw_right + (1 - alpha) * smooth_right;

if detected == 0
    smooth_forward = 0.7 * smooth_forward;
    smooth_right = 0.7 * smooth_right;
end

% Convert normalized command to small position offset
forward_offset = max_position_step * smooth_forward;
right_offset = max_position_step * smooth_right;

% Body-frame to world-frame conversion using yaw
dx_world = cos(yaw) * forward_offset - sin(yaw) * right_offset;
dy_world = sin(yaw) * forward_offset + cos(yaw) * right_offset;

desired_x = x + dx_world;
desired_y = y + dy_world;
desired_z = z;
desired_yaw = yaw;

% Descent disabled for first test
if enable_descent == 1 && state == 3
    desired_z = z - descent_step;
end

cmd_pose = [desired_x; desired_y; desired_z; desired_yaw];

forward_cmd = smooth_forward;
right_cmd = smooth_right;

end