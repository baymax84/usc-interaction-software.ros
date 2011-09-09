function [cmod] = camera_model_read(fname)
% CAMERA_MODEL_READ Read camera model from XML file.
%
%   [cmod] = CAMERA_MODEL_READ(fname) reads a camera model from the file
%   'fname'.  To do so, the function relies on Java XML classes.
%
%   Supported models include CAMERA_LINEAR, CAMERA_OMNIDIR and CAMERA_WARPED.
%   If an attempt is made to load an unknown model type, the function will
%   throw an error.
%
%   If no filename is supplied, the function returns a default model
%   structure for a linear camera with a 640x480 image plane. 
%
%   Inputs:
%   -------
%   [fname]  - Camera model path/filename.
%
%   Outputs:
%   --------
%    cmod  - Struct - camera model.
%
%   See also CAMERA_MODEL_WRITE.

pn = 'rovito:vision:camera';  fn = mfilename;

if nargin == 0
  % Return default model.
  cmod = default_model;
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
  error(errstr(pn, fn, 'modelError'), ...
        'No model type specified in file %s.', fname);
end

model = char(modelList.item(0).getFirstChild.getData);

switch model
  case {'CAMERA_LINEAR', 'CAMERA_WARPED', 'CAMERA_OMNIDIR', 'CAMERA_RANGE'}
    cmod.model = model;
  otherwise
    error(errstr(pn, fn, 'modelError'), 'Unknown model type %s.', model);
end

% Image size (pixels).
imgsizeList = docNode.getElementsByTagName('imgsize');

if imgsizeList.getLength == 0
  error(errstr(pn, fn, 'modelError'), 'Image size missing.');
end

imgsizeNode = imgsizeList.item(0);

try
  w = node_data_to_dbl(imgsizeNode.getElementsByTagName('width').item(0));
  h = node_data_to_dbl(imgsizeNode.getElementsByTagName('height').item(0));
  cmod.imgSize = [w; h];
catch err
  error(errstr(pn, fn, 'modelError'), 'Image width or height missing.');
end

% Forward-Right-Down (optional).
frdList = docNode.getElementsByTagName('frd');

if frdList.getLength ~= 0
  frd = char(frdList.item(0).getFirstChild.getData);
  cmod.frd = sscanf(frd, '%d');
  
  if length(cmod.frd) ~= 3
    error(errstr(pn, fn, 'modelError'), ...
          'FRD specification has wrong number of values.');
  end
end

% Extrinsics (optional).
extrList = docNode.getElementsByTagName('extrinsic');

if extrList.getLength ~= 0
  extrinsic = char(extrList.item(0).getFirstChild.getData);
  cmod.extrinsic = sscanf(extrinsic, '%f');
end

%----- Model-Specific Info -----

% Intrinsics for cameras.
intrList = docNode.getElementsByTagName('intrinsic');
  
if intrList.getLength == 0 && ~strcmp(model, 'CAMERA_RANGE')
  error(errstr(pn, fn, 'modelError'), 'Intrinsic parameters missing.');
end

if strcmp(model, 'CAMERA_LINEAR') || strcmp(model, 'CAMERA_WARPED')
  % Intrinsics for linear cameras.
  intrNode = intrList.item(0);

  try
    % Focal lengths and skew.
    fx = node_data_to_dbl(intrNode.getElementsByTagName('fx').item(0));
    fy = node_data_to_dbl(intrNode.getElementsByTagName('fy').item(0));
    sx = node_data_to_dbl(intrNode.getElementsByTagName('sx').item(0));

    % Image center.
    cx = node_data_to_dbl(intrNode.getElementsByTagName('cx').item(0));
    cy = node_data_to_dbl(intrNode.getElementsByTagName('cy').item(0));    
    
    cmod.intrinsic = camera_linear_intr_mat(fx, sx, cx, fy, cy);
  catch err
    error(errstr(pn, fn, 'modelError'), ...
          'One or more intrinsic parameters missing.');
  end

  % Distortion parameters for warped camera.
  if strcmp(model, 'CAMERA_WARPED')
    distortList = docNode.getElementsByTagName('distortion');

    if distortList.getLength == 0
      error(errstr(pn, fn, 'modelError'), 'Distortion parameters missing.');
    end

    distortion = char(distortList.item(0).getFirstChild.getData);
    cmod.distortion = sscanf(distortion, '%f');
  end
end

if strcmp(model, 'CAMERA_OMNIDIR')
  % Intrinsics for omnidirectional cameras.
  intrNode = intrList.item(0);
 
  try
    % Affine parameters.
    ac = node_data_to_dbl(intrNode.getElementsByTagName('ac').item(0));
    ad = node_data_to_dbl(intrNode.getElementsByTagName('ad').item(0));
    ae = node_data_to_dbl(intrNode.getElementsByTagName('ae').item(0));
    
    % Image center.
    cx = node_data_to_dbl(intrNode.getElementsByTagName('cx').item(0));
    cy = node_data_to_dbl(intrNode.getElementsByTagName('cy').item(0));   
    
    cmod.intrinsic = camera_omnidir_intr_mat(ac, ad, ae, cx, cy);
  catch err
    error(errstr(pn, fn, 'modelError'), ...
          'One or more intrinsic parameters missing.');
  end

  % Z axis polynomial.
  polynomialList = docNode.getElementsByTagName('polynomial');

  if polynomialList.getLength == 0
    error(errstr(pn, fn, 'modelError'), ...
          'Z axis polynomial coefficients missing.');
  end

  polynomial = char(polynomialList.item(0).getFirstChild.getData);
  cmod.polynomial = sscanf(polynomial, '%f');
  
  % Valid image region (inner/outer radii).
  radiiList = docNode.getElementsByTagName('radius');
  
  if radiiList.getLength == 0
   error(errstr(pn, fn, 'modelError'), 'Image radii missing.');
  end

  radiiNode = radiiList.item(0);
  
  try
    inner = node_data_to_dbl(radiiNode.getElementsByTagName('inner').item(0));
    outer = node_data_to_dbl(radiiNode.getElementsByTagName('outer').item(0)); 
  catch err
    error(errstr(pn, fn, 'modelError'), ...
          'Inner or outer image radius missing.');
  end

  cmod.radius = [inner; outer];
end

end % camera_model_read

%-------------------------------------------------------------------------------

function [cmod] = default_model
% Return default camera model structure (linear).

cmod.model = 'CAMERA_LINEAR';
cmod.imgSize = [640; 480];
cmod.intrinsic = camera_linear_intr_mat(550, 0, 320, 550, 240);

end

function [value] = node_data_to_dbl(node)
% Retrieve double value from node.

value = str2double(char(node.getFirstChild.getData));

end  % node_data_to_dbl