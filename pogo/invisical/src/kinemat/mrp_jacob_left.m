function [drdp] = mrp_jacob_left(p, q)
% MRP_JACOB_LEFT Jacobian of product of modified Rodrigues parameters.
% 
%   [drdp] = MRP_JACOB_LEFT(p, q) computes the Jacobian of the MRP product
%   vector 'r' with respect to vector 'p'.
%
%   Inputs:
%   -------
%    p  - 3x1 vector of modified Rodrigues parameters.
%    q  - 3x1 vector of modified Rodrigues parameters.
%
%   Outputs:
%   --------
%    drdp  - 3x3 matrix, partial derivatives drdp, of r = p*q w.r.t. p.

p2 = p.'*p;                                 % Magnitude of p^2.
q2 = q.'*q;                                 % Magnitude of q^2.

n = (1 - p2)*q + (1 - q2)*p - 2*skew(q)*p;  % Numerator.
d = (1 + p2*q2 - 2*p.'*q);                  % Denominator.

% Partials w.r.t. numerator.
dndp(:, 1) = [ 1 - 2*p(1)*q(1) - q2; 
              -2*q(3) - 2*p(1)*q(2); 
               2*q(2) - 2*p(1)*q(3)];
             
dndp(:, 2) = [ 2*q(3) - 2*p(2)*q(1);
               1 - 2*p(2)*q(2) - q2;
              -2*q(1) - 2*p(2)*q(3)];

dndp(:, 3) = [-2*q(2) - 2*p(3)*q(1);
               2*q(1) - 2*p(3)*q(2);
               1 - 2*p(3)*q(3) - q2];

% Partials w.r.t. denominator.
dddp = 2*(p*q2 - q).';

% Full Jacobian.
drdp = (dndp*d - n*dddp)/d^2;