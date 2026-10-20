function [command, landed] = descendController(dronePos, image, padCenter, params)

    [height, width, ~] = size(image);

    imageCenter = [width/2, height/2];

    centerError = padCenter - imageCenter;

    errorX = centerError(1);
    errorY = centerError(2);

    vx = params.kp_image * errorY;
    vy = params.kp_image * errorX;
    vz = -params.descentSpeed;

    command = [vx, vy, vz];

    command = max(min(command, params.maxSpeed), -params.maxSpeed);

    z = dronePos(3);

    landed = z <= params.landingHeight;

end