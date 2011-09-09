function [d] = angle_sub(a, b)
% ANGLE_SUB Difference between two angles.
% 
%   [d] = ANGLE_SUB(a, b) computes the normalized difference between angle 
%   vectors 'a' and 'b', i.e. a - b.
%
%   Inputs:
%   -------
%    a  - nx1 vector of angles, in radians.
%    b  - nx1 vector of angles, in radians.
%
%   Outputs:
%   --------
%    d  - nx1 vector of normalized angle differences, a - b.
%
%   See also ANGLE_NORMALIZE.

d = angle_normalize(a - b);