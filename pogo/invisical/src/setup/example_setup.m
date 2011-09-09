function [setup] = example_setup

%-----------------------------------------------------------------------------
% Example Setup
%-----------------------------------------------------------------------------

setup.name = 'Camera-IMU Calibration Setup - Example File';
setup.desc = 'Description goes here, if desired.';

% Files and directories.
baseDir = '/Users/jonathan/Academic/PhD/Research/datasets/camimu/2009-08-09_rth/';

setup.data.poseFile  = fullfile(pwd, 'setup/mrp_pose_001.mat');
setup.data.imuFile   = fullfile(baseDir, 'fullcal-1/fullcal-1.imu');
setup.data.ptsDir    = fullfile(baseDir, 'fullcal-1/points');
setup.data.imgDir    = fullfile(baseDir, 'fullcal-1/rectified');

setup.files.cmodFile = fullfile(baseDir, 'camcal/linear_model_left.xml');
setup.files.imodFile = fullfile(baseDir, 'imucal/imu_3DM-GX3_2.xml');
setup.files.gmodFile = fullfile(baseDir, 'imucal/gravity_los_angeles.xml');

% Timing, stepping and sequencing.
setup.data.camDelay = 0.049;           % Estimated camera latency.
setup.data.camSteps = 1;               % Use every nth image only.
setup.data.firstImg =  745;            % First image for calibration.
setup.data.finalImg = 1600;            % Final image for calibration.

% Iterated filtering.
setup.ispkf.maxIters = 25;             % Maximum number of ISPKF iterations.
setup.ispkf.deltaTol = 0.001;          % Tolerance for sequential iterations.

% Plot settings (ununsed).
setup.plot.xlimits = [0.0, 150.0];     % Domain - timestamps.
setup.plot.ylimits = ...
[
   -0.075,   0.075;                    % Range - x translation.
    0.075,   0.225;                    % Range - y translation.
   -0.075,   0.075;                    % Range - z translation.
       87,      93;                    % Range - roll.
       -3,       3;                    % Range - pitch.
      -93,     -87;                    % Range - yaw.
];