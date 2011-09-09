function [pose, covar] = initial_pose_imu(Qwc, Swc, Qic, Sic)
% INITIAL_POSE_IMU Initial IMU pose from camera pose and transform.
%
%   [pose, covar] = INITIAL_POSE_IMU(Qwc, Swc, Qic, Sic) computes an initial
%   IMU pose estimate in the world frame, and an estimate of the pose
%   covariance, using the initial camera pose in the global frame and an
%   estimate (guess) of the camera-to-IMU transform.
%
%   The function assumes that the pose covariance matrices 'Swc' and 'Sic' are
%   for the MRP error state representation.  The full covariance matrix 
%   returned includes the IMU pose and the camera-IMU transform (with
%   correlation blocks).
%
%   Inputs:
%   -------
%    Qwc  - 7x1 quaternion pose vector, camera frame wrt world frame.
%    Swc  - 6x6 pose covariance matrix.
%    Qic  - 7x1 quaternion pose vector, camera frame wrt IMU frame.
%    Sic  - 6x6 pose covariance matrix.
%
%   Outputs:
%   --------
%    pose   -   7x1 quaternion pose vector, IMU frame wrt world frame.
%    covar  - 12x12 pose covariance matrix.
%
%   See also INITIAL_POSE_CAM.

Rwc = rotmat_from_quat(Qwc(4:7));
Rci = rotmat_from_quat(Qic(4:7)).';  % Inverse.
  
% IMU pose in global frame.
Rwi = Rwc*Rci;
twi = Qwc(1:3) - Rwi*Qic(1:3);

pose = [twi; quat_from_rotmat(Rwi)];
  
% Stacked pose covariance matrices.
S = [Swc, zeros(6, 6); zeros(6, 6), Sic];

% Derivatives associated with pose.
[dRwcdp0, dRwcdp1, dRwcdp2] = rotmat_jacob_mrp([0; 0; 0]);      % Error state
[dRcidp0, dRcidp1, dRcidp2] = rotmat_jacob_mrp([0; 0; 0]);      % Error state

% Jacobian
J = eye(12);

% Translation
J(1:3, 1:3) =  eye(3);                 % Jtwc
J(1:3, 4)   = -dRwcdp0*Rwi*Qic(1:3);   % Jmwc
J(1:3, 5)   = -dRwcdp1*Rwi*Qic(1:3);
J(1:3, 6)   = -dRwcdp2*Rwi*Qic(1:3);
J(1:3, 7:9) = -Rwi;                    % Jtic
J(1:3, 10)  = -Rwi*dRcidp0*Qic(1:3);   % Jmic
J(1:3, 11)  = -Rwi*dRcidp1*Qic(1:3);
J(1:3, 12)  = -Rwi*dRcidp2*Qic(1:3);

% Rotation
mwi = mrp_product(mrp_from_quat(Qwc(4:7)), -mrp_from_quat(Qic(4:7)));

J(4:6,  4: 6) =  mrp_jacob_left([0; 0; 0], mwi);
J(4:6, 10:12) = -mrp_jacob_right(mwi, [0; 0; 0]);

covar = J*S*J.';