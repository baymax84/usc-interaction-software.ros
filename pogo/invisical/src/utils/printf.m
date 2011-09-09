function printf(varargin)
% PRINTF Write formatted data to stdout.
%
%   PRINTF(format, A, ...) is exactly the same as the standard MATLAB sprintf
%   function, except that it prints directly to the command shell (instead of
%   to a string variable).  All standard sprintf formatting arguments are
%   supported.

disp(sprintf(varargin{:}));