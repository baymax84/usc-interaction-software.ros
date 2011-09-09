function [Xt] = invis_full_state_quat(xt, optargs)
% INVIS_FULL_STATE_QUAT Compute full state vector from error state vector.
%
%   [Xt] = INVIS_FULL_STATE_QUAT(xt, Xt_mean) computes the full state vector
%   for camera-IMU calibration, from the predicted state mean and the current
%   MRP error state vector.
%
%   Inputs:
%   -------
%    xt       - 24x1 error state vector.
%    optargs  - Struct - propagated mean quaternions.
%
%   Outputs:
%   --------
%    Xt  - 26x1 full state vector at time t.

qwi = quat_product(quat_from_mrp(xt( 4: 6)), optargs.qwi_mean);
qic = quat_product(quat_from_mrp(xt(22:24)), optargs.qic_mean);

Xt = [xt(1:3); qwi; xt(7:21); qic];