function [params, dataset, control] = ...
  invis_setup_quat(setup, imodFile, cmodFile, gmodFile)
% INVIS_SETUP_QUAT Setup for camera-IMU calibration.
%
%   [params, dataset, control] = INVIS_SETUP_QUAT(setup, imodFile, ...) sets
%   various standard parameters for camera-IMU relative pose calibration, and
%   loads the IMU and camera data.
%
%   Inputs:
%   -------
%    setup                  - Struct - general calibration settings.
%    setup.data.poseFile    - Camera-IMU relative pose file (initial guess).
%    setup.data.imuFile     - IMU data filename (accels, angular rates).
%    setup.data.ptsDir      - Camera data directory (points).
%
%   [setup.data.imgDir]     - Camera image directory (rectified images).
%   [setup.data.camSteps]   - Use every nth image frame only.
%   [setup.data.firstImg]   - Image number to start processing at.
%   [setup.data.finalImg]   - Image number to finish processing at (inclusive).
%
%   [setup.files.imodFile]  - IMU model file (function argument overrides).
%   [setup.files.cmodFile]  - Camera model file (function argument overrides).
%   [setup.files.gmodFile]  - Gravity model file (function argument overrides).
%
%    imodFile               - IMU model file (noise parameters).
%    cmodFile               - Camera model file (intrinsics).
%    gmodFile               - Gravity model file (local vec, world frame).
%
%   Outputs:
%   --------
%    params   - Struct - Camera, IMU and estimator parameters.
%    dataset  - Struct - Camera and IMU datasets.
%    control  - Struct - Control and visualization flags.
%
%   See also INVIS_CALIB_SEQ.

fn = mfilename;  % MATLAB filename.

%-------------------------------------------------------------------------------
% Set estimator parameters.
%-------------------------------------------------------------------------------
  
% Quaternion orientation representation.  
estParams.initFun = @invis_init_quat;
estParams.propFun = @invis_Xvec_quat;
estParams.measFun = @invis_hvec_cam;

% Additional settings.
estParams.estimator = 'UKF';
estParams.orientRep = 'quat';

% Check for available models.
if nargin < 2, imodFile = setup.files.imodFile;  end
if nargin < 3, cmodFile = setup.files.cmodFile;  end
if nargin < 4, gmodFile = setup.files.gmodFile;  end

%-------------------------------------------------------------------------------
% Load IMU model, gravity model, relative pose settings and IMU data.
%-------------------------------------------------------------------------------

% IMU model.
imod = imu_model_read(imodFile);

imuParams.gyroNoise  = imod.gyroNoise;           % IMU gryo noise stdev.
imuParams.accelNoise = imod.accelNoise;          % IMU accel noise stdev.

imuParams.gyroDrift  = imod.gyroDrift;           % IMU gryo bias drift stdev.
imuParams.accelDrift = imod.accelDrift;          % IMU accel bias drift stdev.

% Gravity model.
gmod = gravity_model_read(gmodFile);

% TODO: Could be in setup file.
imuParams.gravity = gmod.vector;                 % Local gravity, global coords.
imuParams.gravNom = 9.80665;                     % Magnitude of nominal gravity.
imuParams.gravityUncert = [0.10; 0.10; 0.10];    % Gravity uncertainty stdev.

% Relative pose.
clear relPose;  load(setup.data.poseFile);

imuParams.relPose = [relPose.pic; relPose.qic];

 % Relative pose covariance.
imuParams.relPoseCovar = relPose.Sic;

% IMU data.
[imuData, imuTstamp] = ...
  imu_data_read_ascii(setup.data.imuFile, imuParams.gravNom, 'first');

printf('[%s]: Loaded IMU data. Total of %d measurements.', fn, length(imuData));

%-------------------------------------------------------------------------------
% Load camera model and camera data.
%-------------------------------------------------------------------------------

% Camera model.
camParams = camera_model_read(cmodFile);         % Load camera model.
camParams.pixelNoise = [1.0; 1.0];               % Camera pixel noise stdev.
camParams.imgDir = setup.data.imgDir;            % Rectified image directory.

% Adjust offset to compensate for camera latency.
if isfield(setup.data, 'camDelay')
  offset = imuTstamp + setup.data.camDelay;
else
  offset = imuTstamp;
end

% Camera data (from specified directory).
[camData, camTstamp] = invis_load_cam_data(setup.data.ptsDir, offset);

printf('[%s]: Loaded cam data. Total of %d images.', fn, length(camData));
printf('[%s]: First image is %.2f seconds after IMU start.\n', ... 
       fn, camTstamp - offset);

%-------------------------------------------------------------------------------
% Compute parameters, indexes etc.
%-------------------------------------------------------------------------------

% Set start and stop indices in the camera data.
if isfield(setup.data, 'firstImg')
  camParams.startIndex = setup.data.firstImg;
else
  camParams.startIndex = 1;
end

if isfield(setup.data, 'finalImg')
  camParams.stopIndex = setup.data.finalImg;
else
  camParams.stopIndex = length(camData);
end

% Set start and stop indices in IMU data - this assumes that IMU measurements 
% start before and end after camera measurements. We keep a 0.25 s buffer.
imuStartT = camData(camParams.startIndex).tstamp - 0.25;
imuStopT  = camData(camParams.stopIndex).tstamp;

imuParams.startIndex = 1;

while imuData(imuParams.startIndex).tstamp < imuStartT
  imuParams.startIndex = imuParams.startIndex + 1;
end

imuParams.stopIndex = imuParams.startIndex;

while imuData(imuParams.stopIndex).tstamp < imuStopT && ...
      imuParams.stopIndex < length(imuData)
  imuParams.stopIndex = imuParams.stopIndex + 1;
end

printf('[%s]: Processing start: %.2f s, End: %.2f s, Duration: %.2f s', fn, ...
  imuData(imuParams.startIndex).tstamp, imuData(imuParams.stopIndex).tstamp, ...
  imuData(imuParams.stopIndex).tstamp - imuData(imuParams.startIndex).tstamp);

printf('[%s]: Using %d IMU measurements and %d images to calibrate.\n', fn, ...
       imuParams.stopIndex - imuParams.startIndex + 1, ...
       camParams.stopIndex - camParams.startIndex + 1);

% Estimate biases based on the first twenty-five samples.  Assumes
% the IMU is stationary and z-axis is (almost) aligned with local gravity.
% Average over several measurements to discount noise (at the expense of 
% possibly incoporating some - likely negligible - drift).
gyroBias  = zeros(3, 1);
accelBias = zeros(3, 1);

for i = imuParams.startIndex : imuParams.startIndex + 24
  gyroBias  = gyroBias  + imuData(i).rates;
  accelBias = accelBias + imuData(i).accels + imuParams.gravity;
end

imuParams.gyroBias  = gyroBias/25;
imuParams.accelBias = accelBias/25;

printf('[%s]: Estimated gyro. biases: % 6.3f, % 6.3f, % 6.3f', fn, ...
       imuParams.gyroBias);
printf('[%s]: Estimated accl. biases: % 6.3f, % 6.3f, % 6.3f\n', fn, ...
       imuParams.accelBias);

% Compute initial camera pose and uncertainty for target-based case.
[camParams.pose, camParams.poseCovar] = ...
  initial_pose_cam(camParams.intrinsic, ...
                   camData(camParams.startIndex).world, ...
                   camData(camParams.startIndex).image, ...
                   camParams.pixelNoise, 'quat');

printf('[%s]: Estimated cam position: [%6.2f, %6.2f, %6.2f]', fn, ...
       camParams.pose(1:3));

% Determine initial IMU pose and pose uncertainty.
[imuParams.pose, imuParams.poseCovar] = ...
  initial_pose_imu(camParams.pose, camParams.poseCovar,  ...
                   imuParams.relPose, imuParams.relPoseCovar, 'quat');

printf('[%s]: Estimated IMU position: [%6.2f, %6.2f, %6.2f]', fn, ...
       imuParams.pose(1:3));

% Process noise covariance matrix.
estParams.processNoise = diag([  
  imuParams.gyroNoise.^2;                        % Gyroscope noise.
  imuParams.accelNoise.^2;                       % Accelerometer noise.
  imuParams.gyroDrift.^2;                        % Gyroscope drift.
  imuParams.accelDrift.^2;                       % Accelerometer drift.
]);

estParams.camSteps = setup.data.camSteps;
estParams.ispkf = setup.ispkf;

%-------------------------------------------------------------------------------
% Compute initial state for target-based calibration.
%-------------------------------------------------------------------------------

[estParams.initialState, estParams.initialCovar] = ...
  estParams.initFun(imuParams, camParams);

%-------------------------------------------------------------------------------
% Set control flags.
%-------------------------------------------------------------------------------

control.showImagePoints = false;                 % Show image points. 
control.showStateAtStep = 1;                     % Show state every 'n' steps.

%-------------------------------------------------------------------------------
% Assemble data into structs and return.
%-------------------------------------------------------------------------------

params.est = estParams;
params.cam = camParams;    
params.imu = imuParams;

dataset.cam = camData;
dataset.imu = imuData;