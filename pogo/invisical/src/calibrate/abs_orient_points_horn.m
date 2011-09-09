function [Hif, qif] = abs_orient_points_horn(Pi, Pf)
% ABS_ORIENT_POINTS_HORN Absolute orientation from correspondences (Horn).
%
%   [Hif, qif] = ABS_ORIENT_POINTS_HORN(Pi, Pf) estimates the 6-DOF absolute
%   orienation of a body, given a series of 3D point correspondences.  A
%   description of the complete algorithm is given in:
%
%    B. K. P. Horn, "Closed-Form Solution of Absolute Orientation Using Unit
%    Quaternions," Journal of the Optical Society of America A, vol. 4,
%    pp. 629-642, April 1987.
%
%   Note that we use the full solution method here, although the paper gives
%   a simplified solution when all points in one set (or in both sets) are
%   coplanar.
%
%   Inputs:
%   -------
%    Pi  - 3xn array of points (intial).
%    Pf  - 3xn array of points (final).
%
%   Outputs:
%   --------
%    Hif   - 4x4 homogeneous transform matrix, frame 'f' in frame 'i'.
%   [qif]  - 4x1 unit quaternion, frame 'f' wrt frame 'i'.

% Subtract centroids.
Pi0 = mean(Pi, 2);
Pf0 = mean(Pf, 2);

Pi = Pi - repmat(Pi0, 1, ncols(Pi));
Pf = Pf - repmat(Pf0, 1, ncols(Pf));

% Compute M matrix.
M = zeros(3);

for i = 1 : ncols(Pf)
  M = M + Pf(:, i)*(Pi(:, i).');
end

% Compute N matrix.
N = [ ...
  M(1, 1) + M(2, 2) + M(3, 3), ...
  M(2, 3) - M(3, 2), ... 
  M(3, 1) - M(1, 3), ...
  M(1, 2) - M(2, 1);
  M(2, 3) - M(3, 2), ...
  M(1, 1) - M(2, 2) - M(3, 3), ...
  M(1, 2) + M(2, 1), ...
  M(3, 1) + M(1, 3);
  M(3, 1) - M(1, 3), ...
  M(1, 2) + M(2, 1), ...
 -M(1, 1) + M(2, 2) - M(3, 3), ...
  M(2, 3) + M(3, 2);
  M(1, 2) - M(2, 1), ...
  M(3, 1) + M(1, 3), ...
  M(2, 3) + M(3, 2), ...
 -M(1, 1) - M(2, 2) + M(3, 3);
];

% Find largest eigenvalue of N.  The corresponding
% eigenvector is the unit orientation quaternion.
[qif_, d] = eigs(N, 1);

Rif = rotmat_from_quat(qif_);

% Now, compute scale term.
s = sqrt(sum(sum(Pi.^2))/sum(sum(Pf.^2)));

% Finally, compute translation.
T = Pi0 - s*Rif*Pf0;

Hif = [Rif, T; 0, 0, 0, 1];

if nargout == 2
  qif = qif_;
end