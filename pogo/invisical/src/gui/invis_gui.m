function varargout = invis_gui(varargin)
% INVIS_GUI M-file for invis_gui.fig
%    
%   INVIS_GUI, by itself, creates a new CAMIMU_CONTROL or raises the existing
%   singleton*.
%
%   H = INVIS_GUI returns the handle to a new INVIS_GUI or the handle to
%   the existing singleton*.
%
%   INVIS_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%   function named CALLBACK in CAMIMU_CONTROL.M with the given input arguments.
%
%   INVIS_GUI('Property','Value',...) creates a new INVIS_GUI or raises the
%   existing singleton*.  Starting from the left, property value pairs are
%   applied to the GUI before camimu_control_OpeningFunction gets called.  An
%   unrecognized property name or invalid value makes property application
%   stop.  All inputs are passed to camimu_control_OpeningFcn via varargin.
%
%   *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%   instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Last Modified by GUIDE v2.5 12-Jul-2010 14:20:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @invis_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @invis_gui_OutputFcn,  ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before camimu_control is made visible.
function invis_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to invis_gui (see VARARGIN)

% Choose default command line output for invis_gui.
handles.output = hObject;

global control_handle;
control_handle = hObject;

% Set initial application data.
config = struct;
  
% Seven items are required in order to run the calibration algorithm:
%
% 1. MATLAB setup file (.m)
% 2. IMU model     (.xml file)
% 3. Camera model  (.xml file)
% 4. Gravity model (.xml file)
% 5. Relative pose params (.m MATLAB file)
% 6. IMU data (.imu text file)
% 7. Camera data (directory, .cam text files)
%
% The array of flags below indicates which of these items have
% been loaded so far.
config.calFlags = [false, false, false, false, false, false, false];

setappdata(handles.control_panel, 'config', config);

guidata(hObject, handles);

% UIWAIT makes camimu_control wait for user response (see UIRESUME)
% uiwait(handles.control_panel);


% --- Outputs from this function are returned to the command line.
function varargout = invis_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set command line output from handles structure.
varargout{1} = getappdata(handles.control_panel, 'config');

if nargout == 2
  varargout{2} = getappdata(handles.control_panel, 'output');
end


%-------- Calibrate -------

% --- Executes on button press in load_setup_pushbutton.
function load_setup_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to load_setup_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Choose setup file.
[setupFile, setupDir] = uigetfile('*.m', 'Select setup file...');

if setupFile ~= 0
  config = getappdata(handles.control_panel, 'config');
  
  try
    lastDir = pwd;  
    cd(setupDir);
    
    setupFun = str2func(chop_suffix(setupFile));
    setup = setupFun();
    
    cd(lastDir);
    
    printf('[%s]: Loaded setup file.', mfilename);
    printf('[%s]: Name: %s', mfilename, setup.name);

    config.setup = setup;
    
    config.calFlags(1) = true;  % Have valid IMU model now.
        
    if all(config.calFlags)
      set(handles.calibrate_pushbutton, 'Enable', 'on');
    end
  catch
    printf('[%s]: Failed to load setup file, current (if any) unchanged.', ...
           mfilename);
  end
  
  setappdata(handles.control_panel, 'config', config);
end

guidata(hObject, handles);


% --- Executes on button press in load_imu_model_pushbutton.
function load_imu_model_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to load_imu_model_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Choose IMU model file.
[imodFile, imodDir] = uigetfile('*.xml', 'Select IMU model...');

if imodFile ~= 0
  config = getappdata(handles.control_panel, 'config');
  
  try
    config.imu.imodFile = fullfile(imodDir, imodFile);
    config.imu.model = imu_model_read(fullfile(imodDir, imodFile));
    printf('[%s]: Loaded IMU model, type: %s.', ...
           mfilename, config.imu.model.model);

    config.calFlags(2) = true;  % Have valid IMU model now.

    if all(config.calFlags)
      set(handles.calibrate_pushbutton, 'Enable', 'on');
    end
  catch
    printf('[%s]: Failed to load IMU model, current (if any) unchanged.', ...
            mfilename);
  end

  setappdata(handles.control_panel, 'config', config);
end

guidata(hObject, handles);


% --- Executes on button press in load_camera_model_pushbutton.
function load_camera_model_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to load_camera_model_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Choose camera model file.
[cmodFile, cmodDir] = uigetfile('*.xml', 'Select camera model...');

if cmodFile ~= 0
  config = getappdata(handles.control_panel, 'config');

  try
    config.cam.cmodFile = fullfile(cmodDir, cmodFile);
    config.cam.model = camera_model_read(fullfile(cmodDir, cmodFile));
    printf('[%s]: Loaded camera model, type: %s.', ...
           mfilename, config.cam.model.model);

    config.calFlags(3) = true;  % Have valid camera model now.

    if all(config.calFlags)
      set(handles.calibrate_pushbutton, 'Enable', 'on');
    end
  catch
    printf('[%s]: Failed to load camera model, current (if any) unchanged.', ...
            mfilename);
  end
  
  setappdata(handles.control_panel, 'config', config);
end

guidata(hObject, handles);


% --- Executes on button press in load_gravity_model_pushbutton.
function load_gravity_model_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to load_gravity_model_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Choose gravity model file.
[gmodFile, gmodDir] = uigetfile('*.xml', 'Select gravity model...');

if gmodFile ~= 0
  config = getappdata(handles.control_panel, 'config');

  try
    config.grav.gmodFile = fullfile(gmodDir, gmodFile);
    config.grav.model = gravity_model_read(fullfile(gmodDir, gmodFile));
    printf('[%s]: Loaded gravity model, type: %s.', ...
           mfilename, config.grav.model.model);

    config.calFlags(4) = true;  % Have valid gravity model now.

    if all(config.calFlags)
      set(handles.calibrate_pushbutton, 'Enable', 'on');
    end
  catch
    printf('[%s]: Failed to load gravity model, current (if any) unchanged.', ...
            mfilename);
  end
  
  setappdata(handles.control_panel, 'config', config);
end

guidata(hObject, handles);


% --- Executes on button press in load_pose_pushbutton.
function load_pose_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to load_pose_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Next, get name of relative pose file.
[poseFile, poseDir] = uigetfile('*.mat', 'Select pose file...');

if poseFile ~= 0
  config = getappdata(handles.control_panel, 'config');

  clear relPose;
  load(fullfile(poseDir, poseFile));
  
  if ~exist('relPose', 'var')
    printf('[%s]: Bad pose file, try another file.', mfilename);
  else
    config.imu.poseFile = fullfile(poseDir, poseFile);

    config.calFlags(5) = true;  % Have valid pose now.

    if all(config.calFlags)
      set(handles.calibrate_pushbutton, 'Enable', 'on');
    end
  end

  setappdata(handles.control_panel, 'config', config);
end

guidata(hObject, handles);


% --- Executes on button press in load_imu_data_pushbutton.
function load_imu_data_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to load_imu_data_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Choose IMU data file.
[dataFile, dataDir] = uigetfile('*.imu', 'Choose IMU data file...');

if dataFile ~= 0
  config = getappdata(handles.control_panel, 'config');
  
  try
    dataFile = fullfile(dataDir, dataFile);
  
    % config.imu.dataset  = imu_data_read_ascii(dataFile, 9.80665, 'first');
    config.imu.dataFile = dataFile;
    % printf('[%s]: Loaded %d IMU measurements successfully.', ...
    %       mfilename, length(config.imu.dataset));
    
    config.calFlags(6) = 1;  % Have valid IMU data now.
  
    set(handles.load_camera_data_pushbutton, 'Enable', 'on');
  
    if all(config.calFlags)
      set(handles.calibrate_pushbutton, 'Enable', 'on');
    end
  catch
    printf('[%s]: Failed to load IMU data, please try again.', ...
           mfilename);

    clear config.imu.dataset;
    clear config.cam.dataset;
    
    set(handles.load_camera_data_pushbutton, 'Enable', 'off');
  end
  
  setappdata(handles.control_panel, 'config', config);
end

guidata(hObject, handles);


% --- Executes on button press in load_camera_data_pushbutton.
function load_camera_data_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to load_camera_data_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Choose directory from which load camera data.
dataDir = uigetdir('', 'Choose camera data directory (load)...');

% If directory is valid (contains at least one .mat file), save
% list of images and enable 'Locate Target' button.
if dataDir ~= 0
  config = getappdata(handles.control_panel, 'config');
  dataFiles = dir(fullfile(dataDir, '*.mat'));

  if ~isempty(dataFiles)
    config.cam.dataDir = dataDir;

    config.calFlags(7) = 1;  % Have valid camera data now.
         
    if all(config.calFlags)
      set(handles.calibrate_pushbutton, 'Enable', 'on');
    end

    setappdata(handles.control_panel, 'config', config);
  else
    printf('[%s] Directory contains no .mat files.', mfilename);
    set(handles.calibrate_pushbutton, 'Enable', 'off');
  end
end

guidata(hObject, handles);


% --- Executes on button press in calibrate_pushbutton.
function calibrate_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to calibrate_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% TODO: Do we already have calibration output?  If so, ask if the user
% really wants to start over?

config = getappdata(handles.control_panel, 'config');

% Call setup to initialize the required filter parameters.
try
  % Replace pose with selection.
  config.setup.data.poseFile = config.imu.poseFile;

  [config.params, config.dataset, config.control] = ...
    invis_setup_quat(config.setup, ...
                     config.imu.imodFile, ...
                     config.cam.cmodFile, ...
                     config.grav.gmodFile);
catch
  printf('[%s]: Failed to setup filter - try different data files.', ...
          mfilename);
  set(handles.calibrate_pushbutton, 'Enable', 'off');
  return;
end

% Run the actual calibration algorithm, and record the output.
try
  output = ...
    invis_calib_seq(config.params, config.dataset, config.control);
catch
  printf('[%s]: Failed to calibrate - try different data files.', ...
          mfilename);
  set(handles.calibrate_pushbutton, 'Enable', 'off');
  return;
end

set(handles.save_calibration_pushbutton, 'Enable', 'on');
setappdata(handles.control_panel, 'output', output);

guidata(hObject, handles);


% --- Executes on button press in save_calibration_pushbutton.
function save_calibration_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to save_calibration_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

output = getappdata(handles.control_panel, 'output');
uisave(output, 'calibration.mat');


%---------- Visualize ----------

% --- Executes on button press in load_calibration_pushbutton.
function load_calibration_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to load_calibration_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Choose full calibration data file.
[fullcalFile, fullcalDir] = ...
  uigetfile('*.mat', 'Choose (full) calibration file...');

if fullcalFile ~= 0
  config = getappdata(handles.control_panel, 'config');

  % Valid calibration file - must have params, data, and
  % output struct members (at least, there may be others).
  config.fullcal = load(fullfile(fullcalDir, fullcalFile));
  
  if isfield(config.fullcal, 'params')  && ...
     isfield(config.fullcal, 'dataset') && ...
     isfield(config.fullcal, 'output')
    printf('[%s]: Loaded calibration file %s.', mfilename, fullcalFile);
    % set(handles.plot_estimate_pushbutton, 'Enable', 'on');
    % set(handles.image_overlay_pushbutton, 'Enable', 'on');
    
    setappdata(handles.control_panel, 'config', config);
  else
    printf('[%s]: Unable to load %s - invalid calibration file.', ...
           mfilename, fullcalFile);
    
    % set(handles.plot_estimate_pushbutton, 'Enable', 'off');
    % set(handles.image_overlay_pushbutton, 'Enable', 'off');
  end
end

guidata(hObject, handles);


% --- Executes on button press in plot_estimate_pushbutton.
function plot_estimate_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to plot_estimate_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in image_overlay_pushbutton.
function image_overlay_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to image_overlay_pushbutton (see GCBO)
% eventdata  reserved -to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
