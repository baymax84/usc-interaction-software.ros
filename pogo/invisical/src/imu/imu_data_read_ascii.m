function [data, tstamp] = imu_data_read_ascii(fname, gravNom, offset)
% IMU_DATA_READ_ASCII Load IMU data in plain text format from file.
%
%   [data, tstamp] = IMU_DATA_READ_ASCII(fname, gravNom, offset) loads IMU
%   data from the file 'fname'.  Each line of this ASCII file should contain
%   a UNIX timestamp, followed optionally by a timer value (IMU-dependent),
%   and then by three <x, y, z> accelerometer readings and three <x, y, z>
%   rate gyro readings.  Each field is separated by whitespace.
%
%   The 'gravNom' argument is used to convert the accelerometer measurements
%   to the appropriate units (e.g. m/s^2) for processing.  If the value is set
%   to the string 'raw', then no conversion is performed (i.e. the data is
%   loaded in raw format directly from the file).
%
%   The optional 'offset' argument deteremines whether the timestamp for
%   each set of measurements should be adjusted.  If offset is set to the
%   string 'first', all timestamps are adjusted so that the values are
%   relative to the first timestamp in the file, which is set to 0. Otherwise,
%   when a timestamp value is specified, it will be subtracted from each entry.
%   
%   The IMU data is returned as a struct array.  Each entry is a struct
%   containing the timestamp, accelerometer measurements, and gyro 
%   measurements.
%
%   Inputs:
%   -------
%    fname    - Name of file to load IMU data from (full path).
%    gravNom  - Scalar, magnitude of nominal gravity vector or 'raw'.
%   [offset]  - UNIX timestamp value or 'first'.
%
%   Outputs:
%   --------
%    data     - Struct array, with entry for each IMU sample.
%   [tstamp]  - First timestamp in file (before adjustment, if any).

pn = 'rovito:inertial:imu';  fn = mfilename;

M = load(fname);
offset_ = 0;

% Check, correct number of entries per line?
if isempty(M)
  error(errstr(pn, fn, 'fileFormatError'), ...
        'Data file %s is empty.', fname);
end

if ncols(M) ~= 7 && ncols(M) ~= 8
  error(errstr(pn, fn, 'fileFormatError'), ...
        'Data file %s has wrong number of entries per line.', dataFile);
end

if ncols(M) == 7
  acc = 2:4;
  gyr = 5:7;
else
  acc = 3:5;
  gyr = 6:8;
end

if strcmp(gravNom, 'raw')
  gmag = 1.0;
else
  gmag = gravNom;
end

if nargin == 3 
  if strcmp(offset, 'first')
    offset_ = M(1, 1);
  else
    % Otherwise use value specified.
    offset_ = offset;
  end
end

% Pre-allocate for speed.  BUG HERE - not just tstamp...
data(nrows(M)) = struct('tstamp', [], 'tindex', [], 'accels', [], 'rates', []);

% Build struct array.
for i = 1 : size(M, 1)
  data(i).tstamp = M(i, 1) - offset_;       % Relative to offset.

  if ncols(M) == 8
    data(i).tindex = M(i, 2) - M(1, 2);     % Internal timer count.
  end

  data(i).accels = M(i, acc)'*gmag;         % Converted to units.
  data(i).rates  = M(i, gyr)';              % Already in rads/s.
  
  %if size(M, 1) > 1e5 && mod(i, 1000) == 0
  %  printf('[%s]: Processed %d entries.', fn, i);
  %end
end

if nargout == 2
  tstamp = M(1, 1);
end

% Convert IMU internal clock ticks to wall clock values.
% if isfield(data, 'tindex')  %% HACK
%   for i = 1 : length(data)
%     data(i).tstamp = data(i).tindex*1/62500;
%   end
% end