function [dRdp0, dRdp1, dRdp2] = rotmat_jacob_mrp(mrpOrR)
% ROTMAT_JACOB_MRP Jacobian of rotation matrix wrt modified Rodrigues params.
%
%   [dRdp0, dRdp1, dRdp2] = ROTMAT_JACOB_RPY(mrpOrR) computes the Jacobian of
%   of a rotation matrix 'R' with respect to the corresponding modified
%   Rodrigues parameters vector.
%
%   The function will accept either a 3x1 vector of MRPs (and generate R
%   internally), or a 3x3 rotation matrix.
%
%   Inputs:
%   -------
%    mrpOrR  - 3x1 vector of modified Rodrigues parameters, or 3x3
%              orthonormal rotation matrix.
%
%   Outputs:
%   --------
%    dRdp0  - 3x3 matrix of partial derivatives wrt parameter 0.
%    dRdp1  - 3x3 matrix of partial derivatives wrt parameter 1.
%    dRdp2  - 3x3 matrix of partial derivatives wrt parameter 2.

mn = size(mrpOrR);

if(mn(1) == 3 && mn(2) == 3)
  mrp = mrp_from_rotmat(mrpOrR);
else
  mrp = mrpOrR;
end

p0 = mrp(1);
p1 = mrp(2);
p2 = mrp(3);

% Derivatives computed with symbolic toolbox.
d = (1+p0^2+p1^2+p2^2)^3;

dRdp0 = [ ...
  32*(p2^2+p1^2)*p0, ...
  -8*(p2*p0^3+3*p1*p0^2+(-3*p2+p2*p1^2+p2^3)*p0-p1-p1^3-p1*p2^2), ...
   8*(p1*p0^3-3*p2*p0^2+(-3*p1+p1^3+p1*p2^2)*p0+p2+p2*p1^2+p2^3);

   8*(p2*p0^3-3*p1*p0^2+(-3*p2+p2*p1^2+p2^3)*p0+p1+p1^3+p1*p2^2), ...
  16*p0*(-1+p0^2-p1^2+p2^2), ...
   4*(6*p0^2-p0^4-1+p1^4+2*p1^2*p2^2+p2^4-8*p0*p2*p1);

   8*(-p1*p0^3-3*p2*p0^2+(3*p1-p1^3-p1*p2^2)*p0+p2+p2*p1^2+p2^3), ...
  -4*(6*p0^2-p0^4-1+p1^4+2*p1^2*p2^2+p2^4+8*p0*p2*p1), ...
 -16*p0*(1-p0^2-p1^2+p2^2)]/d;
 
dRdp1 = [ ...
  16*p1*(-1-p0^2+p1^2+p2^2), ...
  -8*(-p0^3+p1*p2*p0^2+(-1+3*p1^2-p2^2)*p0-3*p1*p2+p1^3*p2+p1*p2^3), ...
  -4*(6*p1^2-p1^4-1+p0^4+2*p0^2*p2^2+p2^4+8*p0*p2*p1);
  
   8*(p0^3+p1*p2*p0^2+(1-3*p1^2+p2^2)*p0-3*p1*p2+p1^3*p2+p1*p2^3), ...
  32*(p2^2+p0^2)*p1, ...
   8*(-p1*p0^3+p2*p0^2+(3*p1-p1^3-p1*p2^2)*p0+p2-3*p2*p1^2+p2^3);
   
   4*(6*p1^2-p1^4-1+p0^4+2*p0^2*p2^2+p2^4-8*p0*p2*p1), ...
   8*(p1*p0^3+p2*p0^2+(-3*p1+p1^3+p1*p2^2)*p0+p2-3*p2*p1^2+p2^3), ...
 -16*p1*(1-p0^2-p1^2+p2^2)]/d;

dRdp2 = [ ...
  16*p2*(-1-p0^2+p1^2+p2^2), ...
  -4*(-6*p2^2+p2^4+1-p0^4-2*p0^2*p1^2-p1^4+8*p0*p2*p1), ...
   8*(p0^3+p1*p2*p0^2+(1+p1^2-3*p2^2)*p0-3*p1*p2+p1^3*p2+p1*p2^3);
   
   4*(-6*p2^2+p2^4+1-p0^4-2*p0^2*p1^2-p1^4-8*p0*p2*p1), ...
  16*p2*(-1+p0^2-p1^2+p2^2), ...
  -8*(p2*p0^3-p1*p0^2+(-3*p2+p2*p1^2+p2^3)*p0-p1-p1^3+3*p1*p2^2);
  
  -8*(-p0^3+p1*p2*p0^2+(-1-p1^2+3*p2^2)*p0-3*p1*p2+p1^3*p2+p1*p2^3), ...
   8*(p2*p0^3+p1*p0^2+(-3*p2+p2*p1^2+p2^3)*p0+p1+p1^3-3*p1*p2^2), ...
  32*(p1^2+p0^2)*p2]/d;