function [str] = errstr(pathID, funcID, msgID)
% ERRSTR Build and return error-specific identifier string.
%
%   [str] = ERRSTR(pathID, funcID, msgID) builds and returns an error 
%   identifier string for a specific error message.  This is a utility 
%   function.
%
%   Inputs:
%   -------
%    pathID  - Path identifier string.
%    funcID  - Function identifier string.
%    msgID   - Message identifier string.
%
%   Outputs:
%   --------
%    str  - Error string, 'pathID:funcID:msgID' concatenation.

str = strcat(pathID, ':', funcID, ':', msgID);