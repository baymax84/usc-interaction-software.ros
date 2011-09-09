function [p, dpdP, dpdH, dpdK] = camera_linear_3D_to_2D(K, Hwc, P)
% CAMERA_LINEAR_3D_TO_2D World point to image plane point.
%
%   [p, dpdP, dpdH, dpdK] = CAMERA_LINEAR_3D_TO_2D(K, Hwc, P) computes the
%   projection of the 3D point 'P' onto the camera image plane, for a linear 
%   (distortion-free) camera model.
%
%   Inputs:
%   -------
%    K   - 3x3 camera intrinsic calibration matrix.
%    Hwc - 4x4 homogeneous pose matrix for camera frame wrt world frame.
%    P   - 3x1 point in world coordinate frame.
%
%   Outputs:
%   --------
%    p      - 2x1 projection on image plane (in (u, v) or (col, row) format).
%   [dpdP]  - 2x3 Jacobian of image plane point wrt world point.
%   [dpdH]  - 2x6 Jacobian of image plane point wrt camera pose (Euler RPY).
%   [dpdK]  - 2x5 Jacobian of image plane point wrt intrinsic parameters.
%
%   Each row of the dpdH Jacobian is formatted as:
%
%     [dpdtx, dpdty, dpdtz, dpdroll, dpdpitch, dpdyaw]
%
%   Each row of the dpdK Jacobian is formatted as:
%   
%     [dpdfx, fpdsx, dpdox, dpdfy, dpdoy]
%
%   See also CAMERA_LINEAR_2D_TO_3D.

% Image point is K*R'(P - t)/z in camera frame.

R  = Hwc(1:3, 1:3);
dx = P - Hwc(1:3, 4);

l = K*R'*dx;
g = l(3);

p = l(1:2)/g;  % Scale by depth and discard last row.

%----- Need Jacobians? -----

% dpdP, image point wrt world point.  
if nargout >= 2    
  dldP = K*R.';
  dgdP = dldP(3, :);
  
  dpdP = (g*dldP - l*dgdP)/(g^2);
  dpdP = dpdP(1:2, :);  % Discard last row.
end

% dpdH, image point wrt camera pose.
if nargout >= 3  
  % Translation - using above result.
  dldH(1:3, 1:3) = -dldP;
  
  % Rotation.
  [dRdr, dRdp, dRdq] = rotmat_jacob_rpy(R);

  dldH(1:3, 4) = K*dRdr'*dx;
  dldH(1:3, 5) = K*dRdp'*dx;
  dldH(1:3, 6) = K*dRdq'*dx;
  
  dgdH = dldH(3, :);
  
  dpdH = (g*dldH - l*dgdH)/(g^2);
  dpdH = dpdH(1:2, :);  % Discard last row.
end

% dpdK, image point wrt camera intrinsic parameters.
if nargout == 4
  dpdK = zeros([2, 5]);

  m = R'*dx;
  m = m/m(3);
  
  dpdK(1, 1:3) = m(1:3)';
  dpdK(2, 4:5) = m(2:3)';
end