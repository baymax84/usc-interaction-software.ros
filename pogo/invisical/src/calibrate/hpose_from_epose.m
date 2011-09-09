function [H] = hpose_from_epose(E)
% HPOSE_FROM_EPOSE Homogeneous pose matrix from Euler RPY pose vector.
%
%   [H] = HPOSE_FROM_EPOSE(E) returns the homogeneous pose matrix 'H'
%   corresponding to the Euler RPY pose vector 'E'.  The Euler pose vector 
%   should have the form [x, y, z, r, p, q], where r, p, q are the roll, pitch
%   and yaw Euler angles, respectively.
%
%   Inputs:
%   -------
%    E  - 6x1 Euler RPY pose vector.
%
%   Outputs:
%   --------
%    H  - 4x4 homogeneous pose matrix.

H = [rotmat_from_rpy(E(4:6)), E(1:3); 0, 0, 0, 1];