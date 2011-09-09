function [gmod] = gravity_model_read(fname)
% TARGET_MODEL_READ Read gravity model from XML file.
%
%   [gmod] = GRAVITY_MODEL_READ(fname) reads a gravity model from the file
%   'fname'.  To do so, the function relies on Java XML classes.
%
%   Currently supported models include GRAVITY_SIMPLE only.  If an attempt
%   is made to load an unknown model type, the function will throw an error.
%
%   If no filename is supplied, the function returns a default model
%   structure for the nominal gravity vector of 9.81 m/s^2, straight down.
%
%   Inputs:
%   -------
%   [fname]  - Gravity model path/filename.
%
%   Outputs:
%   --------
%    gmod  - Struct - gravity model.
%
%   See also GRAVITY_MODEL_WRITE.

pn = 'rovito:inertial:utils';
fn = mfilename;

if nargin == 0
  % Return default model.
  gmod = default_model;
  return;
end
  
try
  docNode = xmlread(fname);
catch err
  error(errstr(pn, fn, 'fileReadError'), ...
        'Failed to read XML file %s.', fname);
end

% Type of model.
modelList = docNode.getElementsByTagName('model');

if modelList.getLength == 0
  error('No model type specified in file %s.', fname);
end

model = char(modelList.item(0).getFirstChild.getData);

switch model
  case {'GRAVITY_SIMPLE'}
    gmod.model = model;
  otherwise
    error(errstr(pn, fn, 'modelError'), ...
          'Unknown gravity model type %s.', model);
end

% Magnitudes of components (forward, right, down).
vectList = docNode.getElementsByTagName('vector');

if vectList.getLength == 0
  error(errstr(pn, fn, 'modelError'), 'Gravity vector missing.');
end

vectNode = vectList.item(0);

try
  f = node_data_to_dbl(vectNode.getElementsByTagName('f').item(0));
  r = node_data_to_dbl(vectNode.getElementsByTagName('r').item(0));
  d = node_data_to_dbl(vectNode.getElementsByTagName('d').item(0));

  gmod.vector = [f; r; d];
catch err
  error(errstr(pn, fn, 'modelError'), ...
        'One or more components of gravity vector missing.');
end

% Standard deviations (optional).
uncertList = docNode.getElementsByTagName('uncertainty');

if uncertList.getLength ~= 0
  uncertNode = uncertList.item(0);
  
  try
    fstdev = node_data_to_dbl(uncertNode.getElementsByTagName('fstdev').item(0));
    rstdev = node_data_to_dbl(uncertNode.getElementsByTagName('rstdev').item(0));
    dstdev = node_data_to_dbl(uncertNode.getElementsByTagName('dstdev').item(0));
    
    gmod.uncertainty = [fstdev; rstdev; dstdev];
  catch err
    warning(errstr(pn, fn, 'nonFatal'), ...
            'One or more uncertainty values missing - ignoring.');
  end
end

%----- Model-Specific Info -----

end  % gravity_model_read

%-------------------------------------------------------------------------------

function [gmod] = default_model
% Return default gravity model structure (nominal).

gmod.model = 'GRAVITY_SIMPLE';
gmod.vector = [0.0; 0.0; 9.81];

end

function [value] = node_data_to_dbl(node)
% Retrieve double value from node.

value = str2double(char(node.getFirstChild.getData));

end  % node_data_to_dbl