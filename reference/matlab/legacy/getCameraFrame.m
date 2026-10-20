function image = getCameraFrame()

    imagePath = fullfile("images", "landing_pad_tests", "test1.jpg");

    if isfile(imagePath)
        image = imread(imagePath);
    else
        warning("Test image not found. Returning blank image instead.");
        image = uint8(255 * ones(480, 640, 3));
    end

end