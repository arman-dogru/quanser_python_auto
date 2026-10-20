clear; clc; close all;

addpath("functions");

params = config();

disp("Starting autonomous grid-search landing system...");

% Generate search path
waypoints = generateGridWaypoints(params);
currentWaypointIndex = 1;

mode = "SEARCH";

for t = 1:params.maxIterations

    % 1. Get drone position
    dronePos = estimateDronePosition();

    % 2. Get downward camera image
    image = getCameraFrame();

    % 3. Detect landing pad
    [padDetected, padCenter] = detectLandingPad(image, params);

    % 4. Mode logic
    if mode == "SEARCH"

        if padDetected
            mode = "ALIGN";
            disp("Landing pad detected. Switching to ALIGN mode.");
            command = [0, 0, 0];
        else
            target = waypoints(currentWaypointIndex, :);

            [command, waypointReached] = waypointController(dronePos, target, params);

            if waypointReached
                currentWaypointIndex = currentWaypointIndex + 1;

                if currentWaypointIndex > size(waypoints, 1)
                    disp("Search completed. Landing pad not found.");
                    break;
                end
            end
        end

    elseif mode == "ALIGN"

        [command, centered] = alignToPadController(image, padCenter, params);

        if centered
            mode = "DESCEND";
            disp("Pad centered. Switching to DESCEND mode.");
        end

    elseif mode == "DESCEND"

        [command, landed] = descendController(dronePos, image, padCenter, params);

        if landed
            disp("Landing complete.");
            break;
        end
    end

    % 5. Send command to Quanser drone
    sendDroneCommand(command);

    pause(params.dt);
end

disp("Program finished.");