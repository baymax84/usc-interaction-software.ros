function [camData, firstTstamp] = invis_load_cam_data(dataDir, offset)
% INVIS_LOAD_CAM_DATA Load camera data from file(s).
%
%   [camData] = INVIS_LOAD_CAM_DATA(dataDir, offset) loads camera data from
%   the directory dataDir.  Each file in this directory should be a pre-
%   processed .mat file containing a single structure with timestamp, world
%   points (if available) and image plane projection information.  The files
%   are loaded according to their (OS-dependent) ordering in dataDir, and
%   entries are then sorted according to timestamp.
%
%   The optional argument 'offset' determines whether the timestamp for
%   the measurements should be adjusted.  If 'offset' is set to the string
%   'first', all timestamps are adjusted so that the values are relative to
%   the earliest timestamp for the data.  Otherwise,if a timestamp value is
%   specified, it will be subtracted from each entry.
%
%   The camera data is returned as a struct array.  Each entry is a struct
%   containing the timestamp, known camera world points, and the image plane 
%   projections of those points (in corresponding order).
%
%   Inputs:
%   -------
%    dataDir  - Name of directory to load camera data from (full path).
%   [offset]  - 'first', or UNIX timestamp value.
%
%   Outputs:
%   --------
%    camData       - Struct array, with entries for each camera image.
%   [firstTstamp]  - First camera timestamp (before adjustment, if any).

fl = dir(fullfile(dataDir, '*.mat'));

if size(fl, 1) == 0
  error('Directory has no .mat files for camera data.');
end

% Build struct array.
for i = 1 : length(fl)
  % Data is loaded into struct named 'data'.
  clear data;
  load(fullfile(dataDir, fl(i).name));

  if ~isfield(data, 'image')
      data.image = [];
  end
  
  camData(i) = data;
end

% Sort according to timestamp.
[~, order] = sort([camData(:).tstamp]);
camData = camData(order);

if nargout == 2
  firstTstamp = camData(1).tstamp;
end

% Adjust timestamps if necessary.
if nargin == 2
  if strcmp(offset, 'first')
    offset = camData(1).tstamp;
  end
  
  % Otherwise use numeric value specified.
  for i = 1 : length(camData)
    camData(i).tstamp = camData(i).tstamp - offset;
  end
end