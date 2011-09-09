function [qi] = quat_inverse(q)
% QUAT_INVERSE Quaternion inverse.
%
%   [qi] = QUAT_INVERSE(q) returns the inverse, 'q^-1', of quaternion 'q'.
%
%   Inputs:
%   -------
%    q  - 4x1 quaternion.
%
%   Outputs:
%   --------
%    qi  - 4x1 inverse.

qi = quat_conjugate(q)/(q.'*q);