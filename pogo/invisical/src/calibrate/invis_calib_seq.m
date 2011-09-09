function [output] = invis_calib_seq(params, dataset, control)
% INVIS_CALIB_SEQ Sequential camera-IMU calibration.
%
%   [output] = INVIS_CALIB_SEQ(params, dataset, visuals) performs camera-IMU
%   relative pose calibration using a sequential estimation algorithm.
%
%   Inputs:
%   -------
%    params   - Struct - calibration parameters.
%    dataset  - Struct - calibration dataset.
%    control  - Struct - algorithm control and visualization settings.
%
%   Outputs:
%   --------
%    output  - Struct - posterior state and covariance.

fn = mfilename;  % MATLAB filename.

%-------------------------------------------------------------------------------
% Setup camera data, IMU data.
%-------------------------------------------------------------------------------

% Parameter aliases.
estParams = params.est;
camParams = params.cam;
imuParams = params.imu;

% Data aliases.
camData = dataset.cam;
imuData = dataset.imu;

if isfield(estParams, 'camSteps') && estParams.camSteps > 1
  printf('[%s]: Camera image step size is %d', fn, estParams.camSteps);
else
  estParams.camSteps = 1;
end

if isfield(estParams, 'ispkf')
  maxIters = estParams.ispkf.maxIters;
  deltaTol = estParams.ispkf.deltaTol;
else
  maxIters = 1;
  deltaTol = 0;
end

%-------------------------------------------------------------------------------
% Build data structures and set initial estimator parameters.
%-------------------------------------------------------------------------------

% Choice of filter.
if isfield(estParams, 'estimator') && strcmp(estParams.estimator, 'UKF')
  % Use UKF.
  estimFunc = @invis_ukf;
  
  % UKF parameters (for Gaussian state variables).
  sys.ukfp.alpha  = 10^-3;
  sys.ukfp.beta   =  2.00;
  sys.ukfp.lambda =  0.00;
end

%------------ State Vector ------------

inp.Xti = estParams.initialState;
inp.Pti = estParams.initialCovar;

%--------- System Description ---------

sys.XpropFun = estParams.propFun;

% Process noise covariance matrix.
sys.Q = estParams.processNoise;

%------------ Measurements ------------

meas.hvecFun = estParams.measFun;
meas.hvec_optargs.K = camParams.intrinsic;

% Individual measurement covariance matrix.
R = camParams.pixelNoise.^2;

%---------- Data Structures -----------

output.state = struct('ti', {}, 'tf', {}, 'Xtf', {}, 'Ptf', {});
output.mmnts = struct('tf', {}, 'observed', {}, 'predicted', {});

%----------- Visualization ------------

if control.showImagePoints
  % Show measured/estimated points on image.
  pointsFig = figure;
end

%-------------------------------------------------------------------------------
% Run sequential estimator.
%-------------------------------------------------------------------------------

% Set start indices in dataset.
imuIdx = imuParams.startIndex;
camIdx = camParams.startIndex;

inp.ti = imuData(imuIdx).tstamp;
inp.tf = camData(camIdx).tstamp;

% Step index.
i = 0;

% Print initial state.
invis_pretty_print_state(inp.ti, inp.Xti);

startT = clock;

while camIdx < camParams.stopIndex

  % Select propagate or update step.
  if camIdx > camParams.stopIndex || inp.ti < camData(camIdx).tstamp
    mtype = 'imu';
    
    % Find IMU measurement at or before ti.
    imuPrv = imuIdx;
    while imuData(imuPrv).tstamp > inp.ti
      imuPrv = imuPrv - 1;
    end
    
    % Find IMU measurement at or after tf.
    imuNxt = imuIdx;
    while imuData(imuNxt).tstamp < camData(camIdx).tstamp
      imuNxt = imuNxt + 1;
    end

    % Advance index.
    imuIdx = imuNxt - 1;

    inp.tf = camData(camIdx).tstamp;
    inp.tstamps = [inp.ti, imuData(imuPrv + 1:imuNxt - 1).tstamp, inp.tf];
    
    %-- DEBUG --
    % inp.tstamps
  else
    if ~camData(camIdx).valid
      % Skip this image - propagate IMU forward again.
      printf('[%s]: Skipping camera image %s', fn, camData(camIdx).file);
      camIdx = camIdx + estParams.camSteps;
      continue;
    end

    mtype = 'cam';
    inp.tstamps = camData(camIdx).tstamp;
  end

  %------------------------------------------------------------
  % 1. Propagate state estimate forward in time.
  %------------------------------------------------------------  

  if strcmp(mtype, 'imu')
    % Control inputs.
    sys.Xprop_optargs.w = [imuData(imuPrv:imuNxt).rates];
    sys.Xprop_optargs.a = [imuData(imuPrv:imuNxt).accels];

    [inp.Xti, inp.Pti] = estimFunc(inp, sys);
    
    % State has now been propagated to tf.
  end

  %------------------------------------------------------------
  % 2. Update state estimate with measurement.
  %------------------------------------------------------------  

  if strcmp(mtype, 'cam')
    % Camera measurement available, so perform state update.
    printf('[%s]: Processing image %s', fn, camData(camIdx).file);

    % 2D observations.
    nfeats = ncols(camData(camIdx).image);
    meas.Ztf = reshape(camData(camIdx).image, 2*nfeats, 1);

    if isfield(camData(camIdx), 'indices')
      % Note which landmarks are actually visible.
      meas.indices = camData(camIdx).indices;
    else
      % Assume all landmarks are visible.
      meas.indices = 1:nfeats;
    end

    % Build measurement covariance matrix.
    meas.R = diag(repmat(R, nfeats, 1));

    % We're doing target-based calibration, so include known points.
    meas.hvec_optargs.world = camData(camIdx).world;

    % Iterated unscented filter implementation.
    iter = 0;
    
    while iter < maxIters
      Xtp = inp.Xti;
    
      [inp.Xti, inp.Pti, Ytf] = estimFunc(inp, sys, meas);
    
      if converged(inp.Xti, Xtp, deltaTol), break;  end;
      iter = iter + 1;
    end

    %-- Visualize --
    if control.showImagePoints
      % Show image with overlay of predicted and measured features.
      plot_predicted_vs_measured(meas.Ztf, Ytf, camParams.imgDir, ...
                                 camData(camIdx).file, pointsFig);
    end

    camIdx = camIdx + estParams.camSteps;

    % Update timing information.
    endT = clock;
    elap = etime(endT, startT);
    remn = (camParams.stopIndex - camIdx)*elap/estParams.camSteps/3600;

    printf('[%s]: Step time was %.2f sec. Estimate %.2f hours remain.', ...
           fn, elap, remn);
    
    startT = endT;
    
    % Update performed at tf.
  end
  
  %------------------------------------------------------------
  % 3. Save and print state information (if desired).
  %------------------------------------------------------------
  
  output.state(end + 1) =  ...
    struct('ti', inp.ti, 'tf', inp.tf, 'Xtf', inp.Xti, 'Ptf', inp.Pti);

  if strcmp(mtype, 'cam')
    output.mmnts(end + 1) = ...
      struct('tf', inp.tf, 'observed',  meas.Ztf, 'predicted', Ytf);
  end
                                 
  if mod(i, control.showStateAtStep) == 0
    invis_pretty_print_state(inp.tf, inp.Xti);
  end

  %-- DEBUG --
  % printf('Waiting...');
  % pause;

  % Next loop.
  inp.ti = inp.tf;
  i = i + 1;
end

%-------------------------------------------------------------------------------
% Report results.
%-------------------------------------------------------------------------------

% Print final state.
printf('[%s]: Final state vector:', fn);
invis_pretty_print_state(inp.ti, inp.Xti);
 
end  % invis_calib_seq

%-------------------------------------------------------------------------------

function [yesno] = converged(Xt, Xtp, deltaTol)
% Iterator has converged?

if deltaTol == 0 || norm(Xt(1:7) - Xtp(1:7)) < deltaTol
  yesno = true;
else
  yesno = false;
end

end

function plot_predicted_vs_measured(Ztf, Ytf, imgDir, imgFile, hndl)
% Plot prediction vs. measurement.
      
figure(hndl);  clf;  invy;

imshow(fullfile(imgDir, imgFile));
hold on;

plot_points_2D(reshape(Ztf, 2, nrows(Ztf)/2), 'bo');
plot_points_2D(reshape(Ytf, 2, nrows(Ytf)/2), 'r.');

drawnow;

end  % plot_predicted_vs_measured