function waypoints = generateGridWaypoints(xLimits, yLimits, columns, rows, searchHeight)
%GENERATEGRIDWAYPOINTS Generate an endpoint-preserving serpentine XYZ grid.
arguments
    xLimits (1,2) double
    yLimits (1,2) double
    columns (1,1) double {mustBeInteger,mustBePositive}
    rows (1,1) double {mustBeInteger,mustBePositive}
    searchHeight (1,1) double
end

assert(xLimits(1) < xLimits(2), 'xLimits must be [minimum maximum].');
assert(yLimits(1) < yLimits(2), 'yLimits must be [minimum maximum].');

x = linspace(xLimits(1), xLimits(2), columns);
y = linspace(yLimits(1), yLimits(2), rows);
waypoints = zeros(columns * rows, 3);

next = 1;
for row = 1:rows
    if mod(row, 2) == 1
        rowX = x;
    else
        rowX = fliplr(x);
    end
    indices = next:(next + columns - 1);
    waypoints(indices, :) = [rowX(:), repmat(y(row), columns, 1), ...
        repmat(searchHeight, columns, 1)];
    next = next + columns;
end
end
