function [xtf, qwi_mean, qic_mean] = invis_Xvec_quat(tstamps, xti, wt, optargs)
% INVIS_XVEC_QUAT Propagate state vector.
%
%   [Xt] = INVIS_XVEC_QUAT(tstamps, xti, wt, optargs) computes the camera-IMU 
%   state vector at time tf, according to the state propagation equations, 
%   given the state vector at time ti < tf. The solution is computed by 
%   numerical integration (MATLAB's ODE45 function).
%
%   This version of the function uses the quaternion parameterization for the 
%   global IMU orientation and the camera-IMU orientation.  Note that the 
%   function expects, propagates and returns the error state vector, and not
%   the full state vector.
%
%   The 'optargs' structure must contain the following fields:
%
%    optargs.qwi_ti  - 4x1 orientation quaternion at time ti.
%    optargs.qic_ti  - 4x1 orientation quaternion at time ti.
%    optargs.a       - 3xn measured IMU accelerations at each timestamp.
%    optargs.w       - 3xn measured IMU angular rates at each timestamp.
%
%   If 'optargs' contains the fields:
%
%    optargs.qwi_mean  - Propagated mean quaternion.
%    optargs.qic_mean  - Propagated mean quaternion.
%
%   the propagated mean quaternions are used to calculate the propagated
%   error quaternions.  Otherwise, the propagated error quaternions are the
%   identify quaternions.
%
%   Inputs:
%   -------
%    tstamps  - Array of timestamp values, ti = tstamps(1), tf = tstamps(end).
%    xti      - 24x1 error state vector at time ti.
%    wt       - 12x1 state noises at time ti.
%
%   Outputs:
%   --------
%    xtf        - 24x1 error state vector at time tf.
%   [qwi_mean]  - Propagated mean quaternion.
%   [qic_mean]  - Propagated mean quaternion.
%
%   See also ODE45.

%-------------------------------------------------------------------------------

% Nested ODE routine.
function [Xdot] = Xvec_ode(t, Xt)
  Xdot = zeros(16, 1);

  % Select appropriate control inputs.
  idx = find(tstamps <= t, 1, 'last');

  % 3x1 vector, measured IMU rotation rate minus bias minus noise.
  w = optargs.w(:, idx) - Xt(11:13) - wt(1:3);  
  
  % 3x1 vector, measured IMU linear acceleration minus bias minus noise.
  a = optargs.a(:, idx) - Xt(14:16) - wt(4:6);

  % Velocity of IMU in global frame (derivative of position).
  Xdot(1:3) = Xt(8:10);
  
  % Rotation rate of IMU frame in global frame.
  Xdot(4:7) = 0.5*[0, -w.'; w, -skew(w)]*Xt(4:7);

  % Acceleration of IMU in global frame (derivative of velocity).
  Xdot(8:10) = rotmat_from_quat(Xt(4:7))*a + xti(16:18);

  % Biases are noise only.
  Xdot(11:16) = wt(7:12);
end  % Xvec_ode

%-------------------------------------------------------------------------------

% If the times are the same, the state vector is the same.
if numel(tstamps) == 1 || abs(tstamps(end) - tstamps(1)) < 1e-13
  xtf = xti;

  if nargout >= 2, qwi_mean = optargs.qwi_ti;  end
  if nargout >= 3, qic_mean = optargs.qic_ti;  end

  return
end

% Convert error state (MRP) orientation representation 
% to full quaternion representation for propagation.
qwi = quat_product(quat_from_mrp(xti(4:6)), optargs.qwi_ti);
Xti = [xti(1:3); qwi; xti(7:15)];

% Demand increased accuracy from the integrator.
odeopts = odeset('RelTol', 1e-8, 'AbsTol', 1e-9);

[t, Xt] = ode45(@Xvec_ode, tstamps, Xti, odeopts);
Xtf = Xt(end, :).';

if isfield(optargs, 'qwi_mean')
  qwi_err = quat_product(Xtf(4:7), quat_inverse(optargs.qwi_mean));
else
  qwi_err = quat_identity;   % Identify quaternion.
  qwi_mean = Xtf(4:7);
end

% Do this here, for lack of a better place (at the moment).
if ~isfield(optargs, 'qic_mean')
  qic_mean = optargs.qic_ti;
end

% Assemble updated error state vector.
xtf = xti;
xtf(1:15) = [Xtf(1:3); mrp_from_quat(qwi_err); Xtf(8:16)];

end  % invis_Xvec_quat