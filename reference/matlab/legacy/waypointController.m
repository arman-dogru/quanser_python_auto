function [command, waypointReached] = waypointController(dronePos, target, params)

    error = target - dronePos;

    command = params.kp_waypoint * error;

    command = max(min(command, params.maxSpeed), -params.maxSpeed);

    waypointReached = norm(error) < params.waypointTolerance;

end