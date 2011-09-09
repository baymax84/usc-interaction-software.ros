function [drdq] = mrp_jacob_right(p, q)
% MRP_JACOB_RIGHT Jacobian of product of modified Rodrigues parameters.
%
%   [drdq] = MRP_JACOB_RIGHT(p, q) computes the Jacobian of the MRP product
%   vector 'r' with respect to vector 'q'.
%
%   Inputs:
%   -------
%    p  - 3x1 vector of modified Rodrigues parameters.
%    q  - 3x1 vector of modified Rodrigues parameters.
%
%   Outputs:
%   --------
%    drdq  - 3x3 matrix, partial derivatives drdq, of r = p*q w.r.t. q.

p2 = p.'*p;                                 % Magnitude of p^2.
q2 = q.'*q;                                 % Magnitude of q^2.

n = (1 - p2)*q + (1 - q2)*p - 2*skew(q)*p;  % Numerator.
d = (1 + p2*q2 - 2*p.'*q);                  % Denominator.

% Partials w.r.t. numerator.
dndq(:, 1) = [ 1 - 2*q(1)*p(1) - p2;
               2*p(3) - 2*p(2)*q(1);
              -2*p(2) - 2*p(3)*q(1)];

dndq(:, 2) = [-2*p(3) - 2*p(1)*q(2);
               1 - 2*q(2)*p(2) - p2;
               2*p(1) - 2*p(3)*q(2)];
            
dndq(:, 3) = [ 2*p(2) - 2*p(1)*q(3);
              -2*p(1) - 2*p(2)*q(3);
               1 - 2*q(3)*p(3) - p2];
                   
% Partials w.r.t. denominator.
dddq = 2*(p2*q - p)';

% Full Jacobian.
drdq = (dndq*d - n*dddq)/d^2;