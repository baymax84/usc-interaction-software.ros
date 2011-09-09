function [imod] = imu_model_read(fname)
% IMU_MODEL_READ Read IMU model from XML file.
%
%   [imod] = IMU_MODEL_READ(fname) reads an IMU model from the file 'fname'.
%   To do so, the function relies on Java XML classes.
%
%   Currently supported models include IMU_3AXIS_BASIC only.  If an attempt
%   is made to load an unknown model type, the function will throw an error.
%
%   If no filename is supplied, the function returns a default model
%   structure for a basic 3-axis IMU. 
%
%   Inputs:
%   -------
%   [fname]  - IMU model path/filename.
%
%   Outputs:
%   --------
%    imod  - Struct - IMU model.
%
%   See also IMU_MODEL_WRITE.

pn = 'rovito:inertial:imu';
fn = mfilename;

if nargin == 0
  % Return default model.
  imod = default_model;
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
  case {'IMU_3AXIS_BASIC'}
    imod.model = model;
  otherwise
    error(errstr(pn, fn, 'modelError'), ...
          'Unknown IMU model type %s.', model);
end

%----- Model-Specific Info -----

if strcmp(model, 'IMU_3AXIS_BASIC')
  % Sensor noise.
  noiseList = docNode.getElementsByTagName('noise');
  
  if noiseList.getLength == 0
    error(errstr(pn, fn, 'modelError'), 'Noise parameters missing.');
  end
  
  noiseNode = noiseList.item(0);
 
  try
    % Gyroscope noise.
    gyroNoiseNode = noiseNode.getElementsByTagName('gyroscope').item(0); 
    x = node_data_to_dbl(gyroNoiseNode.getElementsByTagName('xstdev').item(0));
    y = node_data_to_dbl(gyroNoiseNode.getElementsByTagName('ystdev').item(0));
    z = node_data_to_dbl(gyroNoiseNode.getElementsByTagName('zstdev').item(0));
    
    imod.gyroNoise = [x; y; z];
  catch err
    error(errstr(pn, fn, 'modelError'), ...
          'One or more gyroscope noise parameters missing.');
  end
 
  try
    % Accelerometer noise.
    accelNoiseNode = noiseNode.getElementsByTagName('accelerometer').item(0);
    x = node_data_to_dbl(accelNoiseNode.getElementsByTagName('xstdev').item(0));
    y = node_data_to_dbl(accelNoiseNode.getElementsByTagName('ystdev').item(0));
    z = node_data_to_dbl(accelNoiseNode.getElementsByTagName('zstdev').item(0));
    
    imod.accelNoise = [x; y; z];
  catch err
    error(errstr(pn, fn, 'modelError'), ...
          'One or more accelerometer noise parameters missing.');
  end

  % Sensor drift.
  driftList = docNode.getElementsByTagName('drift');
  
  if driftList.getLength == 0
    error(errstr(pn, fn, 'modelError'), 'Drift parameters missing.');
  end
  
  driftNode = driftList.item(0);
 
  try
    % Gyroscope bias drift.
    gyroDriftNode = driftNode.getElementsByTagName('gyroscope').item(0); 
    x = node_data_to_dbl(gyroDriftNode.getElementsByTagName('xstdev').item(0));
    y = node_data_to_dbl(gyroDriftNode.getElementsByTagName('ystdev').item(0));
    z = node_data_to_dbl(gyroDriftNode.getElementsByTagName('zstdev').item(0));
    
    imod.gyroDrift = [x; y; z];
  catch err
    error(errstr(pn, fn, 'modelError'), ...
          'One or more gyroscope drift parameters missing.');
  end
 
  try
    % Accelerometer bias drift.
    accelDriftNode = driftNode.getElementsByTagName('accelerometer').item(0);
    x = node_data_to_dbl(accelDriftNode.getElementsByTagName('xstdev').item(0));
    y = node_data_to_dbl(accelDriftNode.getElementsByTagName('ystdev').item(0));
    z = node_data_to_dbl(accelDriftNode.getElementsByTagName('zstdev').item(0));
    
    imod.accelDrift = [x; y; z];
  catch err
    error(errstr(pn, fn, 'modelError'), ...
          'One or more accelerometer drift parameters missing.');
  end
end

end  % imu_model_read

%-------------------------------------------------------------------------------

function [imod] = default_model
% Return default IMU model structure (basic, no noise or drift).

imod.model = 'IMU_3AXIS_BASIC';

imod.gyroNoise  = [0.0; 0.0; 0.0];
imod.accelNoise = [0.0; 0.0; 0.0];
imod.gyroDrift  = [0.0; 0.0; 0.0];
imod.accelDrift = [0.0; 0.0; 0.0];

end

function [value] = node_data_to_dbl(node)
% Retrieve double value from node.

value = str2double(char(node.getFirstChild.getData));

end  % node_data_to_dbl