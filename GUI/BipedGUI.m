%============================GUI FUNCTIONS================================
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
%      applied to the GUI be fore BipedGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to BipedGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help BipedGUI

% Last Modified by GUIDE v2.5 09-Feb-2017 21:38:45

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

axes(handles.axes3)
matlabImage = imread('Caminante.png');
image(matlabImage)
axis off
axis image
javaaddpath('/home/gardamerinos/drake-distro/build/install/share/java/lcm.jar')
javaaddpath('/home/gardamerinos/drake-distro/biped/build/biped_lcm/biped_lcm.jar')

global lc msg aggregator logAggr tableData1 tableData2 teensies indexMap

teensies = containers.Map({0, 1, 2, 3},{'UR_', 'UL_', 'LR_', 'LL_'});
indexMap = containers.Map({1,2,3,4,5,6,7,8,9,10,11,12},{1,7,2,8,3,9,10,11,12,4,5,6});
lc = lcm.lcm.LCM.getSingleton();
msg = biped_lcm.commData2Teensy;

aggregator = lcm.lcm.MessageAggregator();
logAggr = lcm.lcm.MessageAggregator();

for i = 0:3
    value = values(teensies,{i});
    lc.subscribe(strcat(value,'cmd_response'), aggregator);     
    lc.subscribe(strcat(value,'log_msg'), logAggr);
end     

tableData1 = zeros(4,6);
tableData2 = zeros(4,6);

% --- Outputs from this function are returned to the command line.
function varargout = BipedGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

%=========================FUNCTIONS=======================================
function checkLogMsgLoop(handles)
    global logAggr
    msgIN = logAggr.getNextMessage(1); %Wait milliseconds
    if length(msgIN) > 0
        decoded = biped_lcm.log_msg(msgIN.data);
        set(handles.State,'string',strcat('State:  ',char(decoded.msg)));
    end
    
function output = specifyTeensy(joint_num)
    global teensies
    num = floor((double(joint_num)-1)/3);
    output = values(teensies,{num});

% --- Executes on button press in initialize.
function initialize_Callback(hObject, eventdata, handles)
    global lc msg aggregator tableData1 tableData2 indexMap
    set(handles.State,'string','State: Getting IDs');
    joint_num = get(handles.joint_number,'string');
    joint_num = int8(str2num(joint_num));
    msg.command = biped_lcm.commData2Teensy.ID_REQUEST;
    for i = 0:3
        teensy = specifyTeensy(i*3+1);
        lc.publish(strcat(teensy,'cmd_in'), msg);  
        msgIN = aggregator.getNextMessage(25); %Wait milliseconds
        checkLogMsgLoop(handles);
        decoded = biped_lcm.commDataFromTeensy(msgIN.data);
        
        for j = 1:3
            if (cell2mat(values(indexMap,{decoded.joints(j)})) < 7) 
                tableData1(1:4,(mod(cell2mat(values(indexMap,{decoded.joints(j)}))-1,6)+1)) = [(decoded.angle(j));(decoded.minPot(j));(decoded.maxPot(j));(decoded.joints(j))];
                set(handles.leftLegTable,'Data',tableData1);
                set(handles.leftLegTable,'RowName',{'Angle';'Min Angle'; 'Max Angle'; 'Status'})
            else 
                tableData2(1:4,(mod(cell2mat(values(indexMap,{decoded.joints(j)}))-1,6)+1)) = [(decoded.angle(j));(decoded.minPot(j));(decoded.maxPot(j));(decoded.joints(j))];
                set(handles.rightLegTable,'Data',tableData2);
                set(handles.rightLegTable,'RowName',{'Angle';'Min Angle'; 'Max Angle'; 'Status'})
            end
        end
    end
    
    checkLogMsgLoop(handles);

% --- Executes on button press in startCalibration.
function startCalibration_Callback(hObject, eventdata, handles)
% hObject    handle to startCalibration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global lc msg
set(handles.State,'string','State: Starting Calibration');
checkLogMsgLoop(handles);
joint_num = get(handles.joint_number,'string');
joint_num = int8(str2num(joint_num));
msg.command = biped_lcm.commData2Teensy.START_CALIBRATION;
msg.joint = mod(joint_num-1,3);
teensy = specifyTeensy(joint_num);
lc.publish(strcat(teensy,'cmd_in'), msg);  
checkLogMsgLoop(handles);

% --- Executes on button press in stopCalibration.
function stopCalibration_Callback(hObject, eventdata, handles)
% hObject    handle to stopCalibration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global lc msg aggregator tableData1 tableData2 indexMap
% set(handles.State,'string','State: Stopping Calibration');
checkLogMsgLoop(handles);
joint_num = get(handles.joint_number,'string');
joint_num = int8(str2num(joint_num));
msg.command = biped_lcm.commData2Teensy.STOP_CALIBRATION;
msg.joint = mod(joint_num-1,3);
teensy = specifyTeensy(joint_num);
lc.publish(strcat(teensy,'cmd_in'), msg);  
msgIN = aggregator.getNextMessage(500); %Wait milliseconds
checkLogMsgLoop(handles);
if length(msgIN) > 0
    decoded = biped_lcm.commDataFromTeensy(msgIN.data);
    tableIndex = cell2mat(values(indexMap,{joint_num}));
     if (tableIndex < 7) 
        tableData1(2:3,(mod(tableIndex-1,6)+1)) = [(decoded.minPot(mod(joint_num-1,3)+1));(decoded.maxPot(mod(joint_num-1,3)+1))];
        set(handles.leftLegTable,'Data',tableData1);
        set(handles.leftLegTable,'RowName',{'Angle';'Min Angle'; 'Max Angle'; 'Status'})
    else 
        tableData2(2:3,(mod(tableIndex-1,6)+1)) = [(decoded.minPot(mod(joint_num-1,3)+1));(decoded.maxPot(mod(joint_num-1,3)+1))];
        set(handles.rightLegTable,'Data',tableData2);
        set(handles.rightLegTable,'RowName',{'Angle';'Min Angle'; 'Max Angle'; 'Status'})
    end
end
checkLogMsgLoop(handles);

% --- Executes on button press in runStaticControl.
function runStaticControl_Callback(hObject, eventdata, handles)
% hObject    handle to runStaticControl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global lc msg
set(handles.State,'string','State: Run Static Control');
joint_num = get(handles.joint_number,'string');
joint_num = int8(str2num(joint_num));
joint_drive = get(handles.angle_out,'string');
joint_drive = int16(str2num(joint_drive));
msg.command = biped_lcm.commData2Teensy.RUN_STATIC_CONTROL;
msg.joint = mod(joint_num-1,3);
msg.drive = joint_drive;
teensy = specifyTeensy(joint_num);
lc.publish(strcat(teensy,'cmd_in'), msg);  
while true
  drawnow()
  stop_state = get(handles.stop, 'Value');
  if stop_state
    break;
  end
  checkLogMsgLoop(handles);
end
checkLogMsgLoop(handles);

% --- Executes on button press in staticAll.
function staticAll_Callback(hObject, eventdata, handles)
% hObject    handle to staticAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global msg lc
set(handles.State,'string','State: Run Static All');
joint_num = get(handles.joint_number,'string');
joint_num = int8(str2num(joint_num));
for i = 1:4
    msg.command = biped_lcm.commData2Teensy.STATIC_CONTROL_ALL;
    msg.joint = mod(joint_num-1,3);
    teensy = specifyTeensy(i*3);
    lc.publish(strcat(teensy,'cmd_in'), msg);
end
while true
  drawnow()
  stop_state = get(handles.stop, 'Value');
  if stop_state
    break;
  end
  checkLogMsgLoop(handles);
end
checkLogMsgLoop(handles);

% --- Executes on button press in runTrajectory.
function runTrajectory_Callback(hObject, eventdata, handles)
% hObject    handle to runTrajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global lc msg
set(handles.State,'string','State: Running Trajectory');
joint_num = get(handles.joint_number,'string');
joint_num = int8(str2num(joint_num));
msg.command = biped_lcm.commData2Teensy.RUN_TRAJECTORY;
msg.joint = joint_num;
teensy = specifyTeensy(joint_num);
lc.publish(strcat(teensy,'cmd_in'), msg); 
%Need to send live messages
livemsg = biped_lcm.LiveControlAll;
livemsg.num_joints = 1;
livemsg.joint_ids = 0;
livemsg.angle = 0.1;
lc.publish('live_in',livemsg); 
set(handles.State,'string','State: Ready to Proceed');

% --- Executes on button press in runAll.
function runAll_Callback(hObject, eventdata, handles)
% hObject    handle to runAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global lc msg
set(handles.State,'string','State: Running Trajectory');
joint_num = get(handles.joint_number,'string');
joint_num = int8(str2num(joint_num));
msg.command = biped_lcm.commData2Teensy.RUN_ALL_TRAJECTORIES;
msg.joint = joint_num;
%Need to send live messages
lc.publish('cmd_in', msg);
set(handles.State,'string','State: Ready to Proceed');

% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global msg lc aggregator
set(handles.State,'string','State: STOP');
msg.command = biped_lcm.commData2Teensy.STOP;
for i = 1:4
    teensy = specifyTeensy(i*3);
    lc.publish(strcat(teensy,'cmd_in'), msg);  
    msgIN = aggregator.getNextMessage(10); %Wait milliseconds
    checkLogMsgLoop(handles);
end
checkLogMsgLoop(handles);
set(handles.State,'string','State: Ready to Proceed');

% --- Executes on button press in centralized_control.
function centralized_control_Callback(hObject, eventdata, handles)
% hObject    handle to centralized_control (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%==============================TEXT CALLBACKS==============================
function angle_out_Callback(hObject, eventdata, handles)
% hObject    handle to angle_out (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function angle_out_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angle_out (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function matlab_file_Callback(hObject, eventdata, handles)
% hObject    handle to matlab_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function matlab_file_CreateFcn(hObject, eventdata, handles)
% hObject    handle to matlab_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function joint_number_Callback(hObject, eventdata, handles)
% hObject    handle to joint_number (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function joint_number_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint_number (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function teensy_Callback(hObject, eventdata, handles)
% hObject    handle to teensy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

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
