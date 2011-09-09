function [K] = camera_linear_intr_mat(fx, sx, cx, fy, cy)
% CAMERA_LINEAR_INTR_MAT Intrinsic calibration matrix from parameters.
% 
%   [K] = CAMERA_LINEAR_INTR_MAT(fx, sx, cx, fy, cy) creates a 3x3 
%   intrinsic camera calibration matrix from the specified parameters.
%   The intrinsic calibration matrix is 3x3 upper diagonal.
%
%   Inputs:
%   -------
%    fx   - Camera x focal length.
%    sx   - Skew (theta) factor (usually zero).
%    cx   - x pixel coordinate of principal point.
%    fy   - Camera y focal length.
%    cy   - y pixel coordinate of principal point.
%
%   Outputs:
%   --------
%    K  - 3x3 intrinsic calibration matrix.

K = [fx, sx, cx;
      0, fy, cy;
      0,  0,  1];