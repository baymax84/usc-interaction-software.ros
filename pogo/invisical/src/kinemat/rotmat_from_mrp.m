function [R] = rotmat_from_mrp(mrp)
% ROTMAT_FROM_MRP Rotation matrix from modified Rodrigues parameters.
%
%   [R] = ROTMAT_FROM_MRP(mrp) computes the rotation matrix 'R' from the
%   vector 'mrp' of modified Rodrigues parameters.
%
%   Inputs:
%   -------
%    mrp  - 3x1 vector of modified Rodrigues parameters.
%
%   Outputs:
%   --------
%    R  - 3x3 orthonormal rotation matrix.
%
% Refrences:
%
%   Hanspeter Schaub and John L. Junkins, "Analytical Mechanics of Space
%   Systems", pp 110. AIAA Education Series, 2003.

s  = -skew(mrp);
m2 =  mrp.'*mrp;

R = eye(3) + (8*s*s - 4*(1 - m2)*s)/(1 + m2)^2;