% Octave driver used by tools/make_parity_fixtures.py.
%
% Expects these variables to be defined by the generated case file that
% sources this script:
%   CHART   - one of 'pose', 'recenter', 'extract', 'commloss', 'waypoint',
%             'align', 'descend', 'grid', 'config'
%   FN      - name of the extracted chart function to call (file name stem)
%   ROWS    - numeric matrix, one call per row; column 1 is 1 when a new
%             sequence starts (persistent state is cleared first)
%   DATA    - (extract only) cell array of data vectors, indexed by ROWS(:,2)
% Prints one line per row, values formatted with %.17g, or "ERR <id>" when
% the MATLAB function raises.
warning('off', 'all');
fmt = '%.17g';
for r = 1:rows(ROWS)
  row = ROWS(r, :);
  if row(1) == 1
    clear(FN);
  end
  try
    switch CHART
      case 'pose'
        [cmd_pose, f, rt, st] = feval(FN, [row(2); row(3); row(4); row(5)], row(6), row(7), row(8));
        out = [cmd_pose(:)', f, rt, st];
      case 'recenter'
        [f, rt, d, st] = feval(FN, row(2), row(3), row(4));
        out = [f, rt, d, st];
      case 'extract'
        [p, o, t] = feval(FN, DATA{row(2)}, row(3));
        out = [p(:)', o(:)', t];
      case 'commloss'
        out = feval(FN, row(2), row(3), row(4));
      case 'waypoint'
        params = config();
        [c, reached] = waypointController(row(2:4), row(5:7), params);
        out = [c, double(reached)];
      case 'align'
        params = config();
        img = zeros(row(3), row(2), 3);
        [c, centered] = alignToPadController(img, row(4:5), params);
        out = [c, double(centered)];
      case 'descend'
        params = config();
        img = zeros(row(3), row(2), 3);
        [c, landed] = descendController(row(6:8), img, row(4:5), params);
        out = [c, double(landed)];
      case 'grid'
        params = config();
        params.xMin = row(2); params.xMax = row(3);
        params.yMin = row(4); params.yMax = row(5);
        params.gridSpacing = row(6); params.searchHeight = row(7);
        w = generateGridWaypoints(params);
        out = [rows(w), reshape(w', 1, [])];
    end
    printf('%d', r);
    printf([' ' fmt], out);
    printf('\n');
  catch err
    printf('%d ERR %s\n', r, strrep(err.identifier, ' ', '_'));
  end
end
