function [Ewc, Swc] = camera_pose_ils_rpy(K, Pw, pc, Sc, iters, Hwc0)
% CAMERA_POSE_ILS_RPY Pose from points and projections, iterated least squares.
%
%   [Ewc, Swc] = camera_pose_ils_rpy(K, Pw, pc, Sc, iters, Hwc0) computes the
%   pose of the camera in the world frame, where 'Pw' are a series of (at
%   least six) known world points and 'pc' are their (known) projections into
%   the camera image plane.
%
%   The function uses nonlinear least squares to minimize the reprojection 
%   error of the observed points on the camera image plane.
%
%   Inputs:
%   -------
%    K      - 3x3 camera intrinsic calibration matrix.
%    Pw     - 3xn array of points in world frame.
%    pc     - 2xn array of correspoinding projections onto image plane.
%    Sc     - 2x2xn (or 2xn) array of image point covariance matrices.
%    iters  - Maximum number of optimization iterations.
%   [Hwc0]  - 4x4 homogeneous pose matrix, initial guess (if available).
%
%   Outputs:
%   --------
%    Ewc  - 6x1 Euler RPY pose vector, camera frame wrt world frame.
%    Swc  - 6x6 pose covariance matrix.

fn = mfilename;  % MATLAB filename.

if nargin < 6
  % Compute approximate solution with Horn's algorithm.
  % Hwc0 = camera_pose_quat_horn(K, Pw, pc);
end

% Now, use ILS to minimize pixel reprojection error.
Ewc = epose_from_hpose(Hwc0);

np = ncols(pc);                             % Num points.
ps = reshape(pc, 2*np, 1);                  % Stacked vector of observations.
Ss = zeros(2*np);                           % Stacked block diagonal covariance.

dimSc = ndims(Sc);
  
for i = 1 : np
  l = 2*(i - 1) + 1;  u = l + 1;

  if dimSc == 3
    Ss(l:u, l:u) = Sc(:, :, i);             % Fill in appropriate block.
  else
    Ss(l:u, l:u) = Sc;                      % All covariances are the same.
  end
end

dY = zeros(2*np, 1);                        % Difference.
J  = zeros(2*np, 6);                        % Jacobian.

iter = 1;

while true
  Ewc_prev = Ewc;                           % Save previous pose.
  Hwc = hpose_from_epose(Ewc);              % Homogeneous pose matrix (reuse).
  
  % Project each known point into camera 
  % image, given current pose estimate.
  for i = 1 : ncols(Pw)
    lw = 2*(i - 1) + 1; up = lw + 1;
    [pp, ~, Ji] = camera_linear_3D_to_2D(K, Hwc, Pw(:, i));

    dY(lw:up) = ps(lw:up) - pp;
    J(lw:up, :) = Ji;
  end
  
  % Solve system.
  Swc_ = J.'*(Ss\J);
  dEwc = Swc_\(J.'*(Ss\dY));
  Ewc  = Ewc + dEwc;

  % Converged?
  diff = [Ewc(1:3) - Ewc_prev(1:3); angle_sub(Ewc(4:6), Ewc_prev(4:6))];

  if norm(diff) < 1e-12
    debug_printf(1, '[%s]: Covergence required %d iters.', fn, iter);
    break;
  elseif iter == iters
    debug_printf(1, '[%s]: Failed to converge after %d iters.',  fn, iters);
    break;
  end

  iter = iter + 1;
end

if nargout == 2
  Swc = inv(Swc_);
end