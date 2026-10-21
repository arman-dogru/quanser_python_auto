function tf = contains(str, pattern, varargin)
%CONTAINS Minimal Octave shim for MATLAB's contains (char inputs only).
% Supports contains(str, pattern) and contains(..., 'IgnoreCase', true),
% which is all the original project scripts use.
ignoreCase = false;
for k = 1:2:numel(varargin)
  if strcmpi(varargin{k}, 'IgnoreCase'), ignoreCase = logical(varargin{k+1}); end
end
if ignoreCase, str = lower(str); pattern = lower(pattern); end
tf = ~isempty(strfind(str, pattern));
end
