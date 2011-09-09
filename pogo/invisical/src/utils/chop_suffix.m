function [chop, suff] = chop_suffix(str)
% CHOP_SUFFIX Chop trailing '.<suffix>' from string.
%
%   [chop] = CHOP_SUFFIX(str) chops all characters from the trailing
%   '.' onward in str and returns the chopped result.  Useful for chopping
%   extensions from filenames.
%
%   Inputs:
%   -------
%    str  - String to chop.
%
%   Outputs:
%   --------
%    chop   - Chopped result.  The '.' character is also chopped.
%   [suff]  - Chopped suffix, including the '.' character.

pn = 'rovito:utils:string';
fn = mfilename;

chop = [];

if ~ischar(str)
  error(errstr(pn, fn, 'argumentError'), 'Argument is not a string.');
end

ind = strfind(str, '.');

if size(ind, 1) ~= 0
  chop = str(1:ind(end) - 1);
  
  if nargout == 2
    % Return the suffix as well.
    suff = str(ind(end):end);
  end
end