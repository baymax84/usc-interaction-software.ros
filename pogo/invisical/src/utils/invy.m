function invy
% INVY Invert the current y-axis scale.  
%
%   INVY inverts the y-axis of the current figure.  This is useful
%   for displaying image data, for example.

% Works for current figure only.
set(gca, 'ydir', 'reverse');