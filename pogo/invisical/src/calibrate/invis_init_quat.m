function [Xti, Pti] = invis_init_quat(imuParams, camParams)
% INVIS_INIT_QUAT Initialize state vector for quaternion angle parameterization.
%
%   [Xti, Pti] = INVIS_INIT_QUAT(imuParams, camParams) initializes the
%   calibration state vector, using the quaternion orientation
%   parameterization.  We assume that the camera-IMU platform is stationary
%   at the start of calibration.
%
%   Inputs:
%   -------
%    imuParams  - Struct - IMU parameters (noise characteristics etc.).
%    camParams  - Struct - camera parameters (intrinsics etc.).
%
%   Outputs:
%   --------
%    Xti  - 26x1  initial state vector.
%    Pti  - 24x24 initial error state covariance matrix.

% Initial (full) state vector.
Xti = zeros(26, 1);

% Assume initial velocity is zero.
Xti(1:7)   = imuParams.pose;           % Pose of IMU in global frame.
Xti(11:13) = imuParams.gyroBias;       % Gryoscope biases.
Xti(14:16) = imuParams.accelBias;      % Accelerometer biases.
Xti(17:19) = imuParams.gravity;        % Gravity vector in global frame.
Xti(20:22) = imuParams.relPose(1:3);   % Camera-IMU relative position.
Xti(23:26) = imuParams.relPose(4:7);   % Camera-IMU relative orientation.

% Initial state covariance matrix.
Psti = ...
diag([
  [1e-9; 1e-9; 1e-9];                  % Velocity uncertainty, initially zero.
  imuParams.gyroNoise.^2;              % Accelerometer bias uncertainty.
  imuParams.accelNoise.^2;             % Gyroscope bias uncertainty.
  imuParams.gravityUncert.^2;          % Gravity uncertainty.
]);

Pti = blkdiag(imuParams.poseCovar, Psti, imuParams.relPoseCovar);