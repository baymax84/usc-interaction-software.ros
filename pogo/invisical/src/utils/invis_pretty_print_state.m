function invis_pretty_print_state(t, Xt)
% INVIS_PRETTY_PRINT_STATE Pretty-print camera-IMU state vector.
%
%   INVIS_PRETTY_PRINT_STATE(t, Xt) prints a 'pretty' formatted version of the
%   system state vector Xt at time t.
%
%   Inputs:
%   -------
%    t   - Current time.
%    Xt  - 26x1 state vector at time t.

% Assume 80 column terminal width.
printf('State vector at time t = %.5f\n', t);

printf('pwi:  % 3.5f     qwi:  % 3.5f     vwi:  % 3.5f', Xt(1), Xt(4), Xt(8));
printf('      % 3.5f           % 3.5f           % 3.5f', Xt(2), Xt(5), Xt(9));
printf('      % 3.5f           % 3.5f           % 3.5f', Xt(3), Xt(6), Xt(10));
printf('                         % 3.5f\n', Xt(7));

printf('bg:   % 3.5f      ba:  % 3.5f      gw:  % 3.5f', Xt(11), Xt(14), Xt(17));
printf('      % 3.5f           % 3.5f           % 3.5f', Xt(12), Xt(15), Xt(18));
printf('      % 3.5f           % 3.5f           % 3.5f\n', Xt(13), Xt(16), Xt(19));

printf('pic:  % 3.5f     qic:  % 3.5f', Xt(20), Xt(23));
printf('      % 3.5f           % 3.5f', Xt(21), Xt(24));
printf('      % 3.5f           % 3.5f', Xt(22), Xt(25));
printf('                         % 3.5f\n', Xt(26));