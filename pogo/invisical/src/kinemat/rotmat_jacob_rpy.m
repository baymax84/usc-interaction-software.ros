function [dRdr, dRdp, dRdy] = rotmat_jacob_rpy(rpyOrR)
% ROTMAT_JACOB_RPY Jacobian of rotation matrix wrt Euler RPY angles.
%
%   [dRdr, dRdp, dRdy] = ROTMAT_JACOB_RPY(rpyOrR) computes the Jacobian of
%   a rotation matrix 'R' with respect to the corresponding roll, pitch and
%   yaw Euler angles.
%
%   The function will accept either a 3x1 vector of rpy angles (and 
%   generate R internally), or a 3x3 rotation matrix.
%
%   Inputs:
%   -------
%    rpyOrR  - 3x1 vector of roll, pitch, yaw angles, or 3x3 orthonormal 
%              rotation matrix.
%
%   Outputs:
%   --------
%    dRdr  - 3x3 matrix of partial derivatives wrt roll.
%    dRdp  - 3x3 matrix of partial derivatives wrt pitch.
%    dRdy  - 3x3 matrix of partial derivatives wrt yaw.

mn = size(rpyOrR);

if(mn(1) == 3 && mn(2) == 1)
  R = rotmat_from_rpy(rpyOrR);
  cosy = cos(rpyOrR(3));
  siny = sin(rpyOrR(3));
else
  % Compute cos(rpy(2)) directly from the rotation matrix, avoids trig,
  % uses a sqrt instead.
  R = rpyOrR;
	cosp = sqrt(1 - R(3,1).*R(3,1));
  
  if cosp < 1e-15
    % Choose here?
    cosy = R(1,1)/cosp;
    siny = R(2,1)/cosp;
  else
    cosy = R(1,1)/cosp;
    siny = R(2,1)/cosp;
  end
end

dRdr = R*[ 0,     0,     0;
           0,     0,    -1;
           0,     1,     0];

dRdp = [   0,     0,  cosy; 
           0,     0,  siny;
       -cosy, -siny,     0]*R;

dRdy = [   0,    -1,     0;
           1,     0,     0;
           0,     0,     0]*R;