function [q] = quat_from_mrp(mrp)
% QUAT_FROM_MRP Unit quaternion from modified Rodrigues parameters.
%
%   [q] = QUAT_FROM_MRP(mrp) computes the unit quaternion 'q' from the vector 
%   'mrp' of modified Rodrigues parameters.
%
%   Inputs:
%   -------
%    mrp  - 3x1 vector of modified Rodrigues parameters.
%
%   Outputs:
%   --------
%    q  - 4x1 quaternion with unit norm.

n2 = mrp.'*mrp;

q0 = (1 - n2)/(1 + n2);
qv = 2*mrp/(1 + n2);

q = [q0; qv];