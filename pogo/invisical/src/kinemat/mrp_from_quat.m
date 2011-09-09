function [mrp] = mrp_from_quat(q)
% MRP_FROM_QUAT Modified Rodrigues parameters from unit quaternion.
%
%   [q] = MRP_FROM_QUAT(mrp) computes the vector of modified Rodrigues 
%   parameters from the unit quaternion 'q'.  Note that the function assumes
%   that q has unit norm.
%
%   Inputs:
%   -------
%    q  - 4x1 quaternion with unit norm.
%
%   Outputs:
%   --------
%    mrp  - 3x1 vector of modified Rodrigues parameters.

mrp = q(2:4)/(1 + q(1));