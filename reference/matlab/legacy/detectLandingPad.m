function [padDetected, padCenter] = detectLandingPad(image, params)

    % ============================================================
    % detectLandingPad
    %
    % Detects the landing pad center from a grayscale / RGB image.
    %
    % Main method:
    %   Detect circular black ring structure using imfindcircles.
    %
    % Fallback method:
    %   Detect bright pad body and check for dark markings inside.
    %
    % Outputs:
    %   padDetected = true / false
    %   padCenter   = [u, v] pixel coordinate in ORIGINAL image size
    % ============================================================

    % ------------------------------------------------------------
    % 1. Convert to grayscale
    % ------------------------------------------------------------
    if ndims(image) == 3
        grayImageOriginal = rgb2gray(image);
    else
        grayImageOriginal = image;
    end

    grayImageOriginal = im2double(grayImageOriginal);

    % Original image size
    [origH, origW] = size(grayImageOriginal);

    % ------------------------------------------------------------
    % 2. Resize large images for faster detection
    % ------------------------------------------------------------
    maxDim = max(origH, origW);

    if maxDim > params.maxDetectionImageSize
        scaleFactor = params.maxDetectionImageSize / maxDim;
        grayImage = imresize(grayImageOriginal, scaleFactor);
    else
        scaleFactor = 1;
        grayImage = grayImageOriginal;
    end

    [imgH, imgW] = size(grayImage);
    minDim = min(imgH, imgW);

    % Default output
    padDetected = false;
    padCenter = [NaN, NaN];

    % ------------------------------------------------------------
    % 3. Enhance image
    % ------------------------------------------------------------
    grayEnhanced = imadjust(grayImage);

    % Since pad markings are black, invert image:
    % black markings become bright structures.
    invertedImage = imcomplement(grayEnhanced);

    % ------------------------------------------------------------
    % 4. Circle / ring detection
    % ------------------------------------------------------------
    minRadius = round(params.minCircleRadiusFraction * minDim);
    maxRadius = round(params.maxCircleRadiusFraction * minDim);

    minRadius = max(minRadius, 10);
    maxRadius = max(maxRadius, minRadius + 5);

    try
        [centers, radii, metrics] = imfindcircles( ...
            invertedImage, ...
            [minRadius maxRadius], ...
            "ObjectPolarity", "bright", ...
            "Sensitivity", params.circleSensitivity, ...
            "EdgeThreshold", params.circleEdgeThreshold);
    catch
        centers = [];
        radii = [];
        metrics = [];
    end

    bestScore = -inf;
    bestCenterSmall = [NaN, NaN];

    for k = 1:size(centers, 1)

        c = centers(k, :);
        r = radii(k);

        % Region around circle candidate
        x1 = max(1, round(c(1) - 1.2 * r));
        x2 = min(imgW, round(c(1) + 1.2 * r));
        y1 = max(1, round(c(2) - 1.2 * r));
        y2 = min(imgH, round(c(2) + 1.2 * r));

        roi = grayImage(y1:y2, x1:x2);

        if isempty(roi)
            continue;
        end

        % Candidate validation
        localContrast = prctile(roi(:), 90) - prctile(roi(:), 10);
        darkRatio = mean(roi(:) < params.darkInsideThreshold);
        brightRatio = mean(roi(:) > params.brightPadThreshold);

        if localContrast < params.minLocalContrast
            continue;
        end

        if darkRatio < params.minCircleDarkRatio || darkRatio > params.maxCircleDarkRatio
            continue;
        end

        if brightRatio < params.minCircleBrightRatio
            continue;
        end

        % Score: strong circle + large enough + good contrast
        score = metrics(k) ...
              + 0.5 * (r / minDim) ...
              + 0.5 * localContrast;

        if score > bestScore
            bestScore = score;
            bestCenterSmall = c;
        end
    end

    % If circle detector succeeds, return center
    if bestScore > -inf
        padDetected = true;
        padCenter = bestCenterSmall / scaleFactor;
        return;
    end

    if isfield(params, "useFallbackDetector") && params.useFallbackDetector == false
        padDetected = false;
        padCenter = [NaN, NaN];
        return;
    end

    % ------------------------------------------------------------
    % 5. Fallback: bright pad body detection
    % ------------------------------------------------------------
    graySmooth = imgaussfilt(grayImage, params.gaussianSigma);

    adaptiveThreshold = prctile(graySmooth(:), params.brightPercentile);
    brightThreshold = max(params.brightThresholdMin, adaptiveThreshold);

    brightMask = graySmooth > brightThreshold;

    % Clean mask
    brightMask = bwareaopen(brightMask, params.minBrightArea);
    brightMask = imclose(brightMask, strel("disk", params.brightCloseRadius));
    brightMask = imfill(brightMask, "holes");

    stats = regionprops(brightMask, ...
        "Area", "Centroid", "BoundingBox", "Extent", "Solidity");

    if isempty(stats)
        padDetected = false;
        padCenter = [NaN, NaN];
        return;
    end

    imageArea = imgH * imgW;

    bestScore = -inf;
    bestIdx = NaN;

    for k = 1:length(stats)

        area = stats(k).Area;
        bbox = stats(k).BoundingBox;

        x = round(bbox(1));
        y = round(bbox(2));
        w = round(bbox(3));
        h = round(bbox(4));

        areaFraction = area / imageArea;

        % Reject too small / too large regions
        if areaFraction < params.minPadAreaFraction
            continue;
        end

        if areaFraction > params.maxPadAreaFraction
            continue;
        end

        if w < params.minPadWidth || h < params.minPadHeight
            continue;
        end

        aspectRatio = w / h;

        if aspectRatio < params.minAspectRatio || aspectRatio > params.maxAspectRatio
            continue;
        end

        % Crop candidate region
        x1 = max(1, x);
        y1 = max(1, y);
        x2 = min(imgW, x + w - 1);
        y2 = min(imgH, y + h - 1);

        roiGray = grayImage(y1:y2, x1:x2);
        roiMask = brightMask(y1:y2, x1:x2);

        maskArea = sum(roiMask(:));

        if maskArea == 0
            continue;
        end

        % Check for dark markings inside bright pad body
        darkInside = (roiGray < params.darkInsideThreshold) & roiMask;
        darkRatio = sum(darkInside(:)) / maskArea;

        if darkRatio < params.minDarkInsideRatio
            continue;
        end

        if darkRatio > params.maxDarkInsideRatio
            continue;
        end

        % Score candidate
        score = areaFraction + params.darkRatioWeight * darkRatio;

        if score > bestScore
            bestScore = score;
            bestIdx = k;
        end
    end

    if isnan(bestIdx)
        padDetected = false;
        padCenter = [NaN, NaN];
        return;
    end

    % Return fallback result, scaled back to original image size
    padDetected = true;
    padCenter = stats(bestIdx).Centroid / scaleFactor;

end