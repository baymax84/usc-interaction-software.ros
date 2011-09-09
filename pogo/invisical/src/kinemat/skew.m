function [uhat] = skew(u)
% SKEW Skew-symmetric matrix for cross product.
%
%   [uhat] = SKEW(u) returns a 3x3 skew-symmetric matrix for the 3x1 
%   vector 'u'.  This matrix can be used to compute the cross product
%   u x v as uhat*v (for some other 3x1 vector v).
%
%   Inputs:
%   -------
%    u  - 3x1 vector.
%
%   Outputs:
%   --------
%    uhat  - 3x3 skew-symmetric matrix.

uhat = [    0, -u(3),  u(2);
         u(3),     0, -u(1);
        -u(2),  u(1),    0];