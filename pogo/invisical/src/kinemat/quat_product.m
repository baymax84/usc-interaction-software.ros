function [r] = quat_product(p, q)
% QUAT_PRODUCT Multiplicative product of two quaternions.
%
%   [r] = QUAT_PRODUCT(p, q) returns the product of quaternions 'p' and 'q', 
%   according to quaternion algebra.
%
%   Inputs:
%   -------
%    p  - 4x1 quaternion.
%    q  - 4x1 quaternion.
%
%   Outputs:
%   --------
%    r  - 4x1 product, p*q.

pv = p(2:4);
qv = q(2:4);

r = [p(1)*q(1) - pv.'*qv; p(1)*qv + q(1)*pv + skew(pv)*qv];