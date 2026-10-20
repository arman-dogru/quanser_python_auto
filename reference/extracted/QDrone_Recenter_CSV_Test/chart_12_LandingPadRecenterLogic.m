function [forward_cmd, right_cmd, descend_cmd, state] = LandingPadRecenterLogic(detected, error_x, error_y)

% Inputs:
% detected : 1 if YOLO detects landing pad, 0 otherwise
% error_x  : pad_center_x - image_center_x
% error_y  : pad_center_y - image_center_y

% Outputs:
% forward_cmd : positive = move forward, negative = move backward
% right_cmd   : positive = move right, negative = move left
% descend_cmd : positive small descent command when landing is ready
% state       : 0 no detection, 1 recentering, 2 centered, 3 landing ready

persistent smooth_forward smooth_right centered_counter

if isempty(smooth_forward)
    smooth_forward = 0;
    smooth_right = 0;
    centered_counter = 0;
end

% Tunable parameters
deadband_pixels = 40;
alpha = 0.25;
max_cmd = 0.20;          
centered_required = 10;
descend_speed = 0.05;

raw_forward = 0;
raw_right = 0;
descend_cmd = 0;
state = 0;

% Your image size is 640x480
image_center_x = 320;
image_center_y = 240;

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
        % error_x > 0 -> move backward
        % error_x < 0 -> move forward
        if ~x_centered
            raw_forward = -error_x / image_center_x;
        end

        % error_y > 0 -> move left
        % error_y < 0 -> move right
        if ~y_centered
            raw_right = -error_y / image_center_y;
        end

        raw_forward = min(max(raw_forward, -1), 1);
        raw_right = min(max(raw_right, -1), 1);
    end

    if centered_counter >= centered_required
        state = 3;
        descend_cmd = descend_speed;
        raw_forward = 0;
        raw_right = 0;
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

% Decay commands when no detection
if detected == 0
    smooth_forward = 0.7 * smooth_forward;
    smooth_right = 0.7 * smooth_right;
end

% Scale to small safe command range
forward_cmd = max_cmd * smooth_forward;
right_cmd = max_cmd * smooth_right;

end