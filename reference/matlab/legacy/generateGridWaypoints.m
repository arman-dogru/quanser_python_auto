function waypoints = generateGridWaypoints(params)

    xValues = params.xMin:params.gridSpacing:params.xMax;
    yValues = params.yMin:params.gridSpacing:params.yMax;

    % Make sure the maximum boundaries are included
    if xValues(end) < params.xMax
        xValues = [xValues, params.xMax];
    end

    if yValues(end) < params.yMax
        yValues = [yValues, params.yMax];
    end

    numX = numel(xValues);
    numY = numel(yValues);

    waypoints = zeros(numX * numY, 3);

    rowIndex = 1;

    for i = 1:numY

        y = yValues(i);

        if mod(i, 2) == 1
            xRow = xValues;
        else
            xRow = fliplr(xValues);
        end

        for j = 1:numX
            x = xRow(j);

            waypoints(rowIndex, :) = [x, y, params.searchHeight];

            rowIndex = rowIndex + 1;
        end
    end

end