function [command, centered] = alignToPadController(image, padCenter, params)

    [height, width, ~] = size(image);

    imageCenter = [width/2, height/2];

    centerError = padCenter - imageCenter;

    errorX = centerError(1);
    errorY = centerError(2);


    %Important: depending on camera orientation, you may need to flip
    %signs(-params.kp_image * errorY)
    
    vx = params.kp_image * errorY;
    vy = params.kp_image * errorX;
    vz = 0;

    command = [vx, vy, vz];

    command = max(min(command, params.maxSpeed), -params.maxSpeed);

    centered = norm(centerError) < params.centerTolerancePixels;

end