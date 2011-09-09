function [ht] = invis_hvec_cam(t, xt, optargs)
% INVIS_HVEC_CAM Compute predicted observation from current state.
%
%   [ht] = INVIS_HVEC_CAM(t, xt, optargs) computes the predicted observation
%   vector for the camera-IMU system at time t, given the error state vector
%   at t.  The 'optargs' structure must contain the following fields:
%
%    optargs.qwi_mean  - 4x1 mean orientation quaternion at time t.
%    optargs.qic_mean  - 4x1 mean orientation quaternion at time t.
%    optargs.K         - 3x3 camera intrinsic calibration matrix.
%    optargs.indices   - Indices of 'n' landmark points that are visible.
%
%   For target-based calibration, the 'optargs' structure must also contain
%   the field:
%
%    optargs.world    - 3xn calibration points in global frame.
%
%   The function does *not* check to determine if the projected points are
%   actually visible in the image plane.
%
%   Inputs:
%   -------
%    t   - Time of observation (unused here).
%    xt  - 24x1 error state vector at time t.
%
%   Outputs:
%   --------
%    ht  - 2nx1 vector of predicted observations (image plane points).

qwi = quat_product(quat_from_mrp(xt( 4: 6)), optargs.qwi_mean);
qic = quat_product(quat_from_mrp(xt(22:24)), optargs.qic_mean);

Rwi = rotmat_from_quat(qwi);           % Rotation from IMU to world frame.
Ric = rotmat_from_quat(qic);           % Rotation from camera to IMU frame.

Hwc = eye(4);
Hwc(1:3, 1:3) = Rwi*Ric;
Hwc(1:3, 4) = xt(1:3) + Rwi*xt(19:21);

lndmks = optargs.world;

ht = zeros(2*length(optargs.indices), 1);

% Observations are returned as a vector.
for i = 1 : length(optargs.indices)
  so = 2*i - 1;
  eo =  so + 1;

  ht(so:eo) = ...
    camera_linear_3D_to_2D(optargs.K, Hwc, lndmks(:, optargs.indices(i)));
end