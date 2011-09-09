function [Xtf, Ptf, Ytf, Pyy] = invis_ukf(inp, sys, meas)
% INVIS_UKF Continuous-discrete unscented Kalman filter.
%
%   [Xtf, Ptf, Ytf, Pyy] = INVIS_UKF(inp, sys, meas) implements the continuous/
%   discrete unscented Kalman filter, specifically for the camera-IMU relative
%   pose calibration problem.  Orientations in the state vector are 
%   parameterized by unit quaternions.
%
%   The routine can be used in one of two ways: if a measurement vector 
%   is supplied, then both the state propagation and state update steps 
%   are performed - otherwise, the state is simply propagated forward in 
%   time, without a measurement update.
%
%   The state propagation function 'XpropFun' must have the following
%   signature:
%
%    Xtf = sys.XpropFun(tstamps, Xti, wt, optargs)
%
%   where wt is the process noise vector.  The propagation function should
%   handle any control inputs (if a control is applied).
%
%   The observation prediction function 'hvecFun' must have the following
%   signature:
%
%    ht = meas.hvecFun(tstamp, Xt, optargs)
%
%   Optional arguments can be passed to either of the above functions by 
%   adding a variable to the sys or meas struct (respectively) with the name 
%   '<function>_optargs'.  Control values can be passed in the Xprop_optargs 
%   structure.
%
%   Inputs:
%   -------
%    inp               - Struct - filter inputs at time ti.
%    inp.tstamps       - Propagation timestamps.
%    inp.Xti           - State vector at time ti.
%    inp.Pti           - State covariance matrix at time ti.
%
%    sys               - Struct - system description.
%    sys.XpropFun      - Function - propagate state forward.
%   [sys.Q]            - Process noise covariance matrix.
%   [sys.ukfp.alpha]   - UT spread parameter.
%   [sys.ukfp.beta]    - UT distribution-specific parameter.
%   [sys.ukfp.lambda]  - UT secondary scaling parameter.
%
%   [meas]             - Struct - measurement at time tf.
%    meas.Ztf          - Measurement (observation) vector.
%    meas.R            - Measurement noise covariance matrix.
%    meas.hvecFun      - Function - predicted observation vector.
%
%   Outputs:
%   --------
%    Xtf  - Propagated/updated state vector at time tf.
%    Ptf  - Propagated/updated state covariance matrix at time tf.
%   [Ytf] - Predicted observation vector at time tf (if available).
%   [Pyy] - Predicted observation covariance matrix at time tf.

%----- Setup -----

% Default UKF params.
alpha  = 10^-3;                        % Point spread parameter.
beta   =  2.00;                        % Distribution-specific parameter.
lambda =  0.00;                        % Secondary scaling parameter.

% Set UKF params.
if isfield(sys, 'ukfp')
  if isfield(sys.ukfp, 'alpha'),  alpha  = sys.ukfp.alpha;  end
  if isfield(sys.ukfp, 'beta'),   beta   = sys.ukfp.beta;   end
  if isfield(sys.ukfp, 'lambda'), lambda = sys.ukfp.lambda; end
end

% Copy additional arguments, if any.
Xprop_optargs = sys.Xprop_optargs;

% Store initial orientation quaternions.
Xprop_optargs.qwi_ti = inp.Xti(4:7);
Xprop_optargs.qic_ti = inp.Xti(23:26);

% Compute error state from full state vector.
% Resets the orientation error states to zero.
xti = invis_err_state_quat(inp.Xti);

xdim = nrows(xti);                     % Size of error state vector.
wdim = nrows(sys.Q);                   % Size of process noise vector.

N = xdim + wdim;                       % Dimension of augmented state vector.
nsp = 2*N + 1;                         % Number of sigma points.
kappa = alpha^2*(N + lambda) - N;      % Compound scaling parameter.

Sx = chol(inp.Pti, 'lower');           % Sqrt of initial state covariance.
Sw = chol(sys.Q,   'lower');           % Sqrt of process noice covariance.

% Do we have a measurement?
if nargin == 3
  hvec_optargs = meas.hvec_optargs;
end

%----- Generate Weights -----
 
W = [kappa 0 0.5]/(N + kappa);         % Sigma point weight vector.
W(2) = W(1) + (1 - alpha^2) + beta;
 
%----- Generate Sigma Points -----

% Replicate initial error state mean.
sigmaPts = repmat([xti; zeros(wdim, 1)], 1, nsp);

% Contour martix (columns of the Cholesky matrices).
S = blkdiag(Sx, Sw);

% Compute full sigma points.
sigmaPts(:, 2:nsp) = sigmaPts(:, 2:nsp) + sqrt(N + kappa)*[S, -S];

%----- Predictor -----

sigmaProp = zeros(xdim, nsp);

% Propagate state mean through process model.
[sigmaProp(:, 1), Xprop_optargs.qwi_mean, Xprop_optargs.qic_mean] = ...
  sys.XpropFun(inp.tstamps, sigmaPts(1:xdim, 1), ...
               sigmaPts(xdim + 1:xdim + wdim, 1), Xprop_optargs);

% Propagate sigma points (state and noise) through process model.
parfor i = 2 : nsp
    sigmaProp(:, i) = ...
      sys.XpropFun(inp.tstamps, sigmaPts(1:xdim, i), ...
                   sigmaPts(xdim + 1:xdim + wdim, i), Xprop_optargs);
end

% Error state mean.
xtf = W(1)*sigmaProp(:, 1) + sum(W(3)*sigmaProp(:, 2:nsp), 2);

% Error state difference.
sigmaDiff = sigmaProp - repmat(xtf, 1, nsp);

% Propagated covariance.
Ptf = W(2)*sigmaDiff(:, 1)*sigmaDiff(:, 1).' + ...
      W(3)*sigmaDiff(:, 2:nsp)*sigmaDiff(:, 2:nsp).';

%----- Corrector -----

% If a measurement is available, update state.
if nargin == 3
  mdim = nrows(meas.Ztf);              % Size of measurement vector.
  measProp = zeros(mdim, nsp);

  hvec_optargs.qwi_mean = Xprop_optargs.qwi_mean;
  hvec_optargs.qic_mean = Xprop_optargs.qic_mean;
  hvec_optargs.indices  = meas.indices;

  % Propagate each sigma point (just state) through measurement
  % model.  We include noise (which is purely additive) below.
  parfor i = 1 : nsp
    measProp(:, i) = meas.hvecFun(inp.tstamps, sigmaProp(:, i), hvec_optargs);
  end

  % Mean for vector space quantities.
  Ytf_ = W(1)*measProp(:, 1) + W(3)*sum(measProp(:, 2:nsp), 2);

  measDiff = measProp - repmat(Ytf_, 1, nsp);

  Pxy = W(2)*sigmaDiff(:, 1)*measDiff(:, 1).' + ...
        W(3)*sigmaDiff(:, 2:nsp)*measDiff(:, 2:nsp).';
  Pyy_ = W(2)*measDiff(:, 1)*measDiff(:, 1).' + ...
         W(3)*measDiff(:, 2:nsp)*measDiff(:, 2:nsp).' + meas.R;  % Add noise.

  % Kalman gain.
  K = Pxy/Pyy_;

  % Error state and error covariance update.
  xtf = xtf + K*(meas.Ztf - Ytf_);
  Ptf = Ptf - K*Pyy_*K.';

  if nargout == 3
    Ytf = Ytf_;
  end
  
  if nargout == 4
    Pyy = Pyy_;
  end
end

% Compute full state from error state vector.
Xtf = invis_full_state_quat(xtf, Xprop_optargs);

end  % invis_ukf