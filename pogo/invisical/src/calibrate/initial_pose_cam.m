function [pose, covar] = initial_pose_cam(K, Pw, pc, pixelNoise, orientRep)
% INITIAL_POSE_CAM Initial camera pose in global frame.
%
%   [pose, covar] = INITIAL_POSE_CAM_(K, Pw, pc, pixelNoise, orientRep) compute
%   the initial pose of the camera in the global frame, given a series of image 
%   plane measurements of known calibration target points.  A covariance matrix 
%   for the pose is also returned.
%
%   The pose is computed using an initial linear computation, followed by an
%   iterated least squares refinement.
%
%   Inputs:
%   -------
%    K           - 3x3 camera intrinsic calibration matrix.
%    Pw          - 3xn array of points in world frame.
%    pc          - 2xn array of correspoinding projections on image plane.
%    pixelNoise  - 2x1 vector of x,y image noise standard deviations.
%    orientRep   - Orientation representation ('eulerRPY' or 'quat').
%
%   Outputs:
%   --------
%    pose   - 6x1 or 7x1 Euler RPY/quat. pose vector, camera wrt world frame.
%    covar  - 6x6 pose covariance matrix.
%
%   See also INITIAL_POSE_IMU.

% Assume approximately fronto-parallel camera view of target.
% TODO: This should be converted to use Ansar's algorithm.
Pc  = K\[pc; ones(1, ncols(pc))];
Hwc = abs_orient_points_horn(Pw, Pc);

% Build array of covariance matrices.
Sc = repmat(diag(pixelNoise.^2), [1, 1, size(pc, 2)]);

% Refine using ILS.
if strcmp(orientRep, 'quat')
  [pwc, qwc, covar] = ...
    camera_pose_ils_quat_mrp(K, zeros(3, 1), zeros(2, 1), Pw, pc, Sc, 50, Hwc);
  
  % Convert MRP to quaternion.
  pose = [pwc; qwc];
else
  [pose, covar] = camera_pose_ils_rpy(K, Pw, pc, Sc, 50, Hwc);
end