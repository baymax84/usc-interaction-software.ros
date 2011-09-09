function [h] = plot_points_2D(p, S)
% PLOT_POINTS_2D Plot 2D points in current figure.
%
%   [h] = PLOT_POINTS_2D(p, S) plots the 2D points in array 'p' on the current
%   figure, using the point style S.  If p is empty, the function returns
%   immediately.
%
%   Inputs:
%   -------
%    p   - 2xn array of points.
%   [S]  - Point style (standard MATLAB styles).
%
%   Outputs:
%   --------
%    h  - Handle to line object.

if isempty(p)
  % Ignore - don't plot anything.
  return;
end

if nargin == 1
  h = plot(p(1, :), p(2, :), 'MarkerSize', 10);
else
  h = plot(p(1, :), p(2, :), S, 'MarkerSize', 10, 'LineWidth', 2);
end