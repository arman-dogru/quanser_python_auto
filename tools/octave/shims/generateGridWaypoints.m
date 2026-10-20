function waypoints = generateGridWaypoints(xLimits, yLimits, columns, rows, searchHeight)
%GENERATEGRIDWAYPOINTS Octave shim: the original body verbatim, minus the
% MATLAB-only `arguments` validation block (Octave 9 cannot parse it).
% The original lives unmodified in matlab/generateGridWaypoints.m.
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
