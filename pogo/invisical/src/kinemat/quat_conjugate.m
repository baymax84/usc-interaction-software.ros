function [qc] = quat_conjugate(q)
% QUAT_CONJUGATE Quaternion conjugate.
%
%   [qc] = QUAT_CONJUGATE(q) returns the quaternion conjugate of 'q'.
%
%   Inputs:
%   -------
%    q  - 4x1 quaternion.
%
%   Outputs:
%   --------
%    gc  - 4x1 conjugate.

qc = [q(1); -q(2:4)];