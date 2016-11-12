function varargout = BipedGUI(varargin)
% BIPEDGUI MATLAB code for BipedGUI.fig
%      BIPEDGUI, by itself, creates a new BIPEDGUI or raises the existing
%      singleton*.
%
%      H = BIPEDGUI returns the handle to a new BIPEDGUI or the handle to
%      the existing singleton*.
%
%      BIPEDGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BIPEDGUI.M with the given input arguments.
%
%      BIPEDGUI('Property','Value',...) creates a new BIPEDGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before BipedGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to BipedGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help BipedGUI

% Last Modified by GUIDE v2.5 10-Nov-2016 18:49:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @BipedGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @BipedGUI_OutputFcn, ...
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


% --- Executes just before BipedGUI is made visible.
function BipedGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to BipedGUI (see VARARGIN)

% Choose default command line output for BipedGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes BipedGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = BipedGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in startCalibration.
function startCalibration_Callback(hObject, eventdata, handles)
% hObject    handle to startCalibration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Starting Calibration');
teensy = get(handles.link,'string');
teensy = int8(str2num(teensy));
link = get(handles.link,'string');
link = int8(str2num(link));
matlab_serial_example(2,teensy,link);
set(handles.State,'string','State: Ready to Proceed');


% --- Executes on button press in stopCalibration.
function stopCalibration_Callback(hObject, eventdata, handles)
% hObject    handle to stopCalibration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Stopping Calibration');
teensy = get(handles.link,'string');
teensy = int8(str2num(teensy));
link = get(handles.link,'string');
link = int8(str2num(link));
data = matlab_serial_example(3,teensy,link);
if link == 1
    if data(end-1)~=666   
        set(handles.T1L1_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.T1L1_MAX,'string',strcat('Max: ', num2str(data(end-1))));
    else
        set(handles.T1L1_MIN,'string','Min: ERROR');
        set(handles.T1L1_MAX,'string','Max: ERROR');
    end
elseif link == 2
    if data(end-1)~=666   
        set(handles.T1L2_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.T1L2_MAX,'string',strcat('Max: ', num2str(data(end-1))));
    else
        set(handles.T1L2_MIN,'string','Min: ERROR');
        set(handles.T1L2_MAX,'string','Max: ERROR');
    end
elseif link == 3
    if data(end-1)~=666   
        set(handles.T1L3_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.T1L3_MAX,'string',strcat('Max: ', num2str(data(end-1))));
    else
        set(handles.T1L3_MIN,'string','Min: ERROR');
        set(handles.T1L3_MAX,'string','Max: ERROR');
    end
end
set(handles.State,'string','State: Ready to Proceed');

% --- Executes on button press in sendTrajectory.
function sendTrajectory_Callback(hObject, eventdata, handles)
% hObject    handle to sendTrajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Sending Trajectory');
teensy = get(handles.link,'string');
teensy = int8(str2num(teensy));
link = get(handles.link,'string');
link = int8(str2num(link));
matlab_serial_example(1,teensy,link);
set(handles.State,'string','State: Ready to Proceed');

% --- Executes on button press in runTrajectory.
function runTrajectory_Callback(hObject, eventdata, handles)
% hObject    handle to runTrajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Running Trajectory');
teensy = get(handles.link,'string');
teensy = int8(str2num(teensy));
link = get(handles.link,'string');
link = int8(str2num(link));
matlab_serial_example(0,teensy,link);
set(handles.State,'string','State: Ready to Proceed');

% --- Executes on button press in runStaticControl.
function runStaticControl_Callback(hObject, eventdata, handles)
% hObject    handle to runStaticControl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Run Static Control');
teensy = get(handles.link,'string');
teensy = int8(str2num(teensy));
link = get(handles.link,'string');
link = int8(str2num(link));
matlab_serial_example(4,teensy,link);

% --- Executes on button press in stopStaticControl.
function stopStaticControl_Callback(hObject, eventdata, handles)
% hObject    handle to stopStaticControl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Stop Static Control');
teensy = get(handles.link,'string');
teensy = int8(str2num(teensy));
link = get(handles.link,'string');
link = int8(str2num(link));
matlab_serial_example(5,teensy,link);
set(handles.State,'string','State: Ready to Proceed');



function teensy_Callback(hObject, eventdata, handles)
% hObject    handle to teensy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of teensy as text
%        str2double(get(hObject,'String')) returns contents of teensy as a double


% --- Executes during object creation, after setting all properties.
function teensy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to teensy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function link_Callback(hObject, eventdata, handles)
% hObject    handle to link (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of link as text
%        str2double(get(hObject,'String')) returns contents of link as a double


% --- Executes during object creation, after setting all properties.
function link_CreateFcn(hObject, eventdata, handles)
% hObject    handle to link (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
