function [xt] = invis_err_state_quat(Xt)
% INVIS_ERR_STATE_QUAT Compute error state vector from total state vector.
%
%   [xt] = INVIS_ERR_STATE_QUAT(Xt) computes the error state vector for
%   camera-IMU calibration, from the full state vector.
%
%   Inputs:
%   -------
%    Xt  - 26x1 full state vector.
%
%   Outputs:
%   --------
%    xt  - 24x1 error state vector at time t.

xt = [Xt(1:3); zeros(3, 1); Xt(8:22); zeros(3, 1)];