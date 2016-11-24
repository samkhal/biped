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

% Last Modified by GUIDE v2.5 20-Nov-2016 19:13:43

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
link = get(handles.link,'string');
link = int8(str2num(link));
matlab_serial_example(2,link);
set(handles.State,'string','State: Ready to Proceed');

% --- Executes on button press in sendTrajectory.
function sendTrajectory_Callback(hObject, eventdata, handles)
% hObject    handle to sendTrajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Sending Trajectory');
link = get(handles.link,'string');
link = int8(str2num(link));
matlab_serial_example(1,link);
set(handles.State,'string','State: Ready to Proceed');

% --- Executes on button press in runTrajectory.
function runTrajectory_Callback(hObject, eventdata, handles)
% hObject    handle to runTrajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Running Trajectory');
link = get(handles.link,'string');
link = int8(str2num(link));
matlab_serial_example(0,link);
set(handles.State,'string','State: Ready to Proceed');


% --- Executes on button press in runStaticControl.
function runStaticControl_Callback(hObject, eventdata, handles)
% hObject    handle to runStaticControl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Run Static Control');
link = get(handles.link,'string');
link = int8(str2num(link));
matlab_serial_example(4,link);

% --- Executes on button press in stopStaticControl.
function stopStaticControl_Callback(hObject, eventdata, handles)
% hObject    handle to stopStaticControl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Stop Static Control');
link = get(handles.link,'string');
link = int8(str2num(link));
matlab_serial_example(5,link);
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


% --- Executes on button press in idRequest.
function idRequest_Callback(hObject, eventdata, handles)
% hObject    handle to idRequest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Getting IDs');
link = get(handles.link,'string');
link = int8(str2num(link));
matlab_serial_example(6,link);
set(handles.State,'string','State: Ready to Proceed');


% --- Executes on button press in runAll.
function runAll_Callback(hObject, eventdata, handles)
% hObject    handle to runAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Running Trajectory');
link = get(handles.link,'string');
link = int8(str2num(link));
matlab_serial_example(7,link);
set(handles.State,'string','State: Ready to Proceed');

% --- Executes on button press in stopCalibration.
function stopCalibration_Callback(hObject, eventdata, handles)
% hObject    handle to stopCalibration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.State,'string','State: Stopping Calibration');
link = get(handles.link,'string');
link = int8(str2num(link));
if link~=0
data = matlab_serial_example(3,link);
data = cell2mat(data);
if link == 1
    if data(end-1)~=666   
        set(handles.L1_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.L1_MAX,'string',strcat('Max: ', num2str(data(end-1))));
        set(handles.zero1,'string',strcat('Zero: ', num2str(data(end-3))));
    else
        set(handles.L1_MIN,'string','Min: ERROR');
        set(handles.L1_MAX,'string','Max: ERROR');
        set(handles.zero1,'string','Zero: ERROR');
    end
elseif link == 2
    if data(end-1)~=666   
        set(handles.L2_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.L2_MAX,'string',strcat('Max: ', num2str(data(end-1))));
        set(handles.zero2,'string',strcat('Zero: ', num2str(data(end-3))));
    else
        set(handles.L2_MIN,'string','Min: ERROR');
        set(handles.L2_MAX,'string','Max: ERROR');
        set(handles.zero1,'string','Zero: ERROR');
    end
elseif link == 3
    if data(end-1)~=666   
        set(handles.L3_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.L3_MAX,'string',strcat('Max: ', num2str(data(end-1))));
        set(handles.zero3,'string',strcat('Zero: ', num2str(data(end-3))));
    else
        set(handles.L3_MIN,'string','Min: ERROR');
        set(handles.L3_MAX,'string','Max: ERROR');
        set(handles.zero1,'string','Zero: ERROR');
    end
elseif link == 4
    if data(end-1)~=666   
        set(handles.L4_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.L4_MAX,'string',strcat('Max: ', num2str(data(end-1))));
        set(handles.zero4,'string',strcat('Zero: ', num2str(data(end-3))));
    else
        set(handles.L4_MIN,'string','Min: ERROR');
        set(handles.L4_MAX,'string','Max: ERROR');
        set(handles.zero1,'string','Zero: ERROR');
    end
elseif link == 5
    if data(end-1)~=666   
        set(handles.L5_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.L5_MAX,'string',strcat('Max: ', num2str(data(end-1))));
        set(handles.zero5,'string',strcat('Zero: ', num2str(data(end-3))));
    else
        set(handles.L5_MIN,'string','Min: ERROR');
        set(handles.L5_MAX,'string','Max: ERROR');
        set(handles.zero1,'string','Zero: ERROR');
    end
elseif link == 6
    if data(end-1)~=666   
        set(handles.L6_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.L6_MAX,'string',strcat('Max: ', num2str(data(end-1))));
        set(handles.zero6,'string',strcat('Zero: ', num2str(data(end-3))));
    else
        set(handles.L6_MIN,'string','Min: ERROR');
        set(handles.L6_MAX,'string','Max: ERROR');
        set(handles.zero1,'string','Zero: ERROR');
    end
elseif link == 7
    if data(end-1)~=666   
        set(handles.L7_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.L7_MAX,'string',strcat('Max: ', num2str(data(end-1))));
        set(handles.zero7,'string',strcat('Zero: ', num2str(data(end-3))));
    else
        set(handles.L7_MIN,'string','Min: ERROR');
        set(handles.L7_MAX,'string','Max: ERROR');
        set(handles.zero1,'string','Zero: ERROR');
    end
elseif link == 8
    if data(end-1)~=666   
        set(handles.L8_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.L8_MAX,'string',strcat('Max: ', num2str(data(end-1))));
        set(handles.zero8,'string',strcat('Zero: ', num2str(data(end-3))));
    else
        set(handles.L8_MIN,'string','Min: ERROR');
        set(handles.L8_MAX,'string','Max: ERROR');
        set(handles.zero1,'string','Zero: ERROR');
    end
elseif link == 9
    if data(end-1)~=666   
        set(handles.L9_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.L9_MAX,'string',strcat('Max: ', num2str(data(end-1))));
        set(handles.zero9,'string',strcat('Zero: ', num2str(data(end-3))));
    else
        set(handles.L9_MIN,'string','Min: ERROR');
        set(handles.L9_MAX,'string','Max: ERROR');
        set(handles.zero1,'string','Zero: ERROR');
    end
elseif link == 10
    if data(end-1)~=666   
        set(handles.L10_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.L10_MAX,'string',strcat('Max: ', num2str(data(end-1))));
        set(handles.zero10,'string',strcat('Zero: ', num2str(data(end-3))));
    else
        set(handles.L10_MIN,'string','Min: ERROR');
        set(handles.L10_MAX,'string','Max: ERROR');
        set(handles.zero1,'string','Zero: ERROR');
    end
elseif link == 11
    if data(end-1)~=666   
        set(handles.L11_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.L11_MAX,'string',strcat('Max: ', num2str(data(end-1))));
        set(handles.zero11,'string',strcat('Zero: ', num2str(data(end-3))));
    else
        set(handles.L11_MIN,'string','Min: ERROR');
        set(handles.L11_MAX,'string','Max: ERROR');
        set(handles.zero1,'string','Zero: ERROR');
    end
elseif link == 12
    if data(end-1)~=666   
        set(handles.L12_MIN,'string',strcat('Min: ', num2str(data(end-2))));
        set(handles.L12_MAX,'string',strcat('Max: ', num2str(data(end-1))));
        set(handles.zero12,'string',strcat('Zero: ', num2str(data(end-3))));
    else
        set(handles.L12_MIN,'string','Min: ERROR');
        set(handles.L12_MAX,'string','Max: ERROR');
        set(handles.zero1,'string','Zero: ERROR');
    end
end
else
    for x = 1:12
        data = matlab_serial_example(3,x);
        data = cell2mat(data);
    if x == 1
        if data(end-1)~=666   
            set(handles.L1_MIN,'string',strcat('Min: ', num2str(data(end-2))));
            set(handles.L1_MAX,'string',strcat('Max: ', num2str(data(end-1))));
            set(handles.zero1,'string',strcat('Zero: ', num2str(data(end-3))));
        else
            set(handles.L1_MIN,'string','Min: ERROR');
            set(handles.L1_MAX,'string','Max: ERROR');
            set(handles.zero1,'string','Zero: ERROR');
        end
    elseif x == 2
        if data(end-1)~=666   
            set(handles.L2_MIN,'string',strcat('Min: ', num2str(data(end-2))));
            set(handles.L2_MAX,'string',strcat('Max: ', num2str(data(end-1))));
            set(handles.zero2,'string',strcat('Zero: ', num2str(data(end-3))));
        else
            set(handles.L2_MIN,'string','Min: ERROR');
            set(handles.L2_MAX,'string','Max: ERROR');
            set(handles.zero1,'string','Zero: ERROR');
        end
    elseif x == 3
        if data(end-1)~=666   
            set(handles.L3_MIN,'string',strcat('Min: ', num2str(data(end-2))));
            set(handles.L3_MAX,'string',strcat('Max: ', num2str(data(end-1))));
            set(handles.zero3,'string',strcat('Zero: ', num2str(data(end-3))));
        else
            set(handles.L3_MIN,'string','Min: ERROR');
            set(handles.L3_MAX,'string','Max: ERROR');
            set(handles.zero1,'string','Zero: ERROR');
        end
    elseif x == 4
        if data(end-1)~=666   
            set(handles.L4_MIN,'string',strcat('Min: ', num2str(data(end-2))));
            set(handles.L4_MAX,'string',strcat('Max: ', num2str(data(end-1))));
            set(handles.zero4,'string',strcat('Zero: ', num2str(data(end-3))));
        else
            set(handles.L4_MIN,'string','Min: ERROR');
            set(handles.L4_MAX,'string','Max: ERROR');
            set(handles.zero1,'string','Zero: ERROR');
        end
    elseif x == 5
        if data(end-1)~=666   
            set(handles.L5_MIN,'string',strcat('Min: ', num2str(data(end-2))));
            set(handles.L5_MAX,'string',strcat('Max: ', num2str(data(end-1))));
            set(handles.zero5,'string',strcat('Zero: ', num2str(data(end-3))));
        else
            set(handles.L5_MIN,'string','Min: ERROR');
            set(handles.L5_MAX,'string','Max: ERROR');
            set(handles.zero1,'string','Zero: ERROR');
        end
    elseif x == 6
        if data(end-1)~=666   
            set(handles.L6_MIN,'string',strcat('Min: ', num2str(data(end-2))));
            set(handles.L6_MAX,'string',strcat('Max: ', num2str(data(end-1))));
            set(handles.zero6,'string',strcat('Zero: ', num2str(data(end-3))));
        else
            set(handles.L6_MIN,'string','Min: ERROR');
            set(handles.L6_MAX,'string','Max: ERROR');
            set(handles.zero1,'string','Zero: ERROR');
        end
    elseif x == 7
        if data(end-1)~=666   
            set(handles.L7_MIN,'string',strcat('Min: ', num2str(data(end-2))));
            set(handles.L7_MAX,'string',strcat('Max: ', num2str(data(end-1))));
            set(handles.zero7,'string',strcat('Zero: ', num2str(data(end-3))));
        else
            set(handles.L7_MIN,'string','Min: ERROR');
            set(handles.zero1,'string','Zero: ERROR');
            set(handles.L7_MAX,'string','Max: ERROR');
        end
    elseif x == 8
        if data(end-1)~=666   
            set(handles.L8_MIN,'string',strcat('Min: ', num2str(data(end-2))));
            set(handles.L8_MAX,'string',strcat('Max: ', num2str(data(end-1))));
            set(handles.zero8,'string',strcat('Zero: ', num2str(data(end-3))));
        else
            set(handles.L8_MIN,'string','Min: ERROR');
            set(handles.L8_MAX,'string','Max: ERROR');
            set(handles.zero1,'string','Zero: ERROR');
        end
    elseif x == 9
        if data(end-1)~=666   
            set(handles.L9_MIN,'string',strcat('Min: ', num2str(data(end-2))));
            set(handles.L9_MAX,'string',strcat('Max: ', num2str(data(end-1))));
            set(handles.zero9,'string',strcat('Zero: ', num2str(data(end-3))));
        else
            set(handles.L9_MIN,'string','Min: ERROR');
            set(handles.zero1,'string','Zero: ERROR');
            set(handles.L9_MAX,'string','Max: ERROR');
        end
    elseif x == 10
        if data(end-1)~=666   
            set(handles.L10_MIN,'string',strcat('Min: ', num2str(data(end-2))));
            set(handles.L10_MAX,'string',strcat('Max: ', num2str(data(end-1))));
            set(handles.zero10,'string',strcat('Zero: ', num2str(data(end-3))));
        else
            set(handles.L10_MIN,'string','Min: ERROR');
            set(handles.zero1,'string','Zero: ERROR');
            set(handles.L10_MAX,'string','Max: ERROR');
        end
    elseif x == 11
        if data(end-1)~=666   
            set(handles.L11_MIN,'string',strcat('Min: ', num2str(data(end-2))));
            set(handles.L11_MAX,'string',strcat('Max: ', num2str(data(end-1))));
            set(handles.zero11,'string',strcat('Zero: ', num2str(data(end-3))));
        else
            set(handles.L11_MIN,'string','Min: ERROR');
            set(handles.L11_MAX,'string','Max: ERROR');
            set(handles.zero1,'string','Zero: ERROR');
        end
    elseif x == 12
        if data(end-1)~=666   
            set(handles.L12_MIN,'string',strcat('Min: ', num2str(data(end-2))));
            set(handles.L12_MAX,'string',strcat('Max: ', num2str(data(end-1))));
            set(handles.zero12,'string',strcat('Zero: ', num2str(data(end-3))));
        else
            set(handles.L12_MIN,'string','Min: ERROR');
            set(handles.L12_MAX,'string','Max: ERROR');
            set(handles.zero1,'string','Zero: ERROR');
        end
    end
    drawnow();
    end
end
set(handles.State,'string','State: Ready to Proceed');
