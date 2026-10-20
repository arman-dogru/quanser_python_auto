function parity_driver(chartDir, fixtureDir)
%PARITY_DRIVER Run the MATLAB Function scripts extracted from the Phase 10
% mission-server model on every fixtures/<block>/*.in.csv input sequence and
% write the reference outputs next to it as *.out.csv (%.17g, exact).
%
% Each CSV row is one model tick; the columns are the block inputs in port
% order, flattened. Persistent state is cleared between sequences, exactly as
% a fresh model start would.
blocks = { ...
  'grid',    'chart_23_Grid_Search_Manager.m',              @run_grid; ...
  'confirm', 'chart_65_YOLO_Confirmation_and_Hover_Hold.m', @run_confirm; ...
  'gate',    'chart_82_Landing_Readiness_Gate.m',           @run_gate; ...
  'abort',   'chart_99_Abort_Diagnostic.m',                 @run_abort; ...
  'arbiter', 'chart_114_Autonomous_LAND_Trigger_Arbiter.m', @run_arbiter; ...
  'extract', 'chart_46_Extract_Object_Data.m',              @run_extract; ...
  'mscomm',  'chart_56_MATLAB_Function.m',                  @run_mscomm};
work = tempname(); mkdir(work);
cleaner = onCleanup(@() rmdir_quiet(work));
addpath(work, '-begin');
total = 0;
for b = 1:rows(blocks)
  name = blocks{b,1};
  src = fileread(fullfile(chartDir, blocks{b,2}));
  fn = ['ref_' name];
  src = regexprep(src, '=\s*fcn\(', ['= ' fn '('], 'once');
  fid = fopen(fullfile(work, [fn '.m']), 'w'); fwrite(fid, src); fclose(fid);
  files = dir(fullfile(fixtureDir, name, '*.in.csv'));
  for f = 1:numel(files)
    inFile = fullfile(fixtureDir, name, files(f).name);
    X = dlmread(inFile, ',');
    clear(fn);
    Y = blocks{b,3}(str2func(fn), X);
    outFile = strrep(inFile, '.in.csv', '.out.csv');
    fid = fopen(outFile, 'w');
    fmt = [repmat('%.17g,', 1, columns(Y)-1) '%.17g\n'];
    fprintf(fid, fmt, Y.');
    fclose(fid);
    total = total + rows(X);
  end
  printf('%s: %d sequences\n', name, numel(files));
end
printf('parity_driver: %d ticks evaluated\n', total);
end

function rmdir_quiet(d)
rmpath(d);
if exist(d, 'dir'), confirm_recursive_rmdir(false); rmdir(d, 's'); end
end

function Y = run_grid(f, X)
Y = zeros(rows(X), 14);
for k = 1:rows(X)
  x = X(k,:);
  [d,wi,gc,vc,ca,ce,pe,ds,da] = f(x(1:4)', x(5:8)', x(9:12)', logical(x(13)), x(14:17)', ...
      x(18), x(19), logical(x(20)), x(21));
  Y(k,:) = [d(:)' wi gc vc ca ce pe(:)' ds da];
end
end

function Y = run_confirm(f, X)
Y = zeros(rows(X), 10);
for k = 1:rows(X)
  x = X(k,:);
  [cp,hp,cf,u,v,nps,age] = f(x(1:4)', x(5:10), logical(x(11)), x(12), x(13:16)');
  Y(k,:) = [cp hp(:)' cf u v nps age];
end
end

function Y = run_gate(f, X)
Y = zeros(rows(X), 3);
for k = 1:rows(X)
  x = X(k,:);
  [r,c,rc] = f(x(1), logical(x(2)), logical(x(3)), logical(x(4)), logical(x(5)), x(6), x(7:10)', x(11:14)', x(15));
  Y(k,:) = [r c double(rc)];
end
end

function Y = run_abort(f, X)
Y = zeros(rows(X), 3);
for k = 1:rows(X)
  x = X(k,:);
  [b,a,t] = f(x(1), logical(x(2)), logical(x(3)), x(4), x(5:8)', x(9), x(10));
  Y(k,:) = [b a t];
end
end

function Y = run_arbiter(f, X)
Y = zeros(rows(X), 5);
for k = 1:rows(X)
  x = X(k,:);
  [t,l] = f(x(1:4)', logical(x(5)));
  Y(k,:) = [t(:)' l];
end
end

function Y = run_extract(f, X)
Y = zeros(rows(X), 7);
for k = 1:rows(X)
  x = X(k,:);
  [p,o,t] = f(x(1:7)', x(8));
  Y(k,:) = [p(:)' o(:)' t];
end
end

function Y = run_mscomm(f, X)
Y = zeros(rows(X), 1);
for k = 1:rows(X)
  x = X(k,:);
  Y(k,:) = f(x(1), x(2), x(3));
end
end
