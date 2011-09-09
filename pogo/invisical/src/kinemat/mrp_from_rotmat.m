function [mrp] = mrp_from_rotmat(R)
% MRP_FROM_ROTMAT Modified Rodrigues parameters from rotation matrix.
%
%   [mrp] = MRP_FROM_ROTMAT(R) computes the vector of modified Rodrigues 
%   parameters from the rotation matrix 'R'.
%
%   Inputs:
%   -------
%    R  - 3x3 orthonormal rotation matrix.
%
%   Outputs:
%   --------
%    mrp  - 3x1 vector of modified Rodrigues parameters.

% Recover quaternion first, then determine MRPs.
mrp = mrp_from_quat(quat_from_rotmat(R));