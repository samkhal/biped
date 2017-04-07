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

global r states_temp torques_temp requested lc msg aggregator logAggr liveResponse_LR liveResponse_LL liveResponse_UR liveResponse_UL tableData1 tableData2 teensies indexMap response infoTable

teensies = containers.Map({0, 1, 2, 3},{'UL_', 'UR_', 'LL_', 'LR_'});
indexMap = containers.Map({1,2,3,4,5,6,7,8,9,10,11,12},{1,2,3,7,8,9,4,5,6,10,11,12});
lc = lcm.lcm.LCM.getSingleton();
msg = biped_lcm.commData2Teensy;

aggregator = lcm.lcm.MessageAggregator();
logAggr = lcm.lcm.MessageAggregator();
liveResponse_UR = lcm.lcm.MessageAggregator();
liveResponse_UL = lcm.lcm.MessageAggregator();
liveResponse_LR = lcm.lcm.MessageAggregator();
liveResponse_LL = lcm.lcm.MessageAggregator();

for i = 0:3
    value = values(teensies,{i});
    lc.subscribe(strcat(value,'cmd_response'), aggregator);     
    lc.subscribe(strcat(value,'log_msg'), logAggr);
end     
lc.subscribe('UR_live_out', liveResponse_UR);
lc.subscribe('UL_live_out', liveResponse_UL);
lc.subscribe('LR_live_out', liveResponse_LR);
lc.subscribe('LL_live_out', liveResponse_LL);

tableData1 = zeros(4,6);
tableData2 = zeros(4,6);
response = zeros(3000,12);
requested = zeros(3000,12);
torques_temp = zeros(1,10);
states_temp = zeros(1,10);
infoTable = cell(1,1);

options.floating = true;
options.dt = 0.001;
options.terrain = RigidBodyFlatTerrain;
r = Caminante('urdf/caminante_minimal_gen.urdf',options);
r = compile(r);

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
    global logAggr infoTable
    msgIN = logAggr.getNextMessage(1); %Wait milliseconds
    if length(msgIN) > 0
        decoded = biped_lcm.log_msg(msgIN.data);
        set(handles.State,'string',strcat('State:  ',char(decoded.msg)));
        infoTable{end+1} = decoded.msg;
    end
    
% function checkLogMsg2Loop(handles)
%     global logAggr infoTable
%     msgIN = logAggr.getNextMessage(5); %Wait milliseconds
%     if length(msgIN) >= 3
%         decoded = biped_lcm.log_msg(msgIN.data);
%         for i = 1:3
%             infoTable(1,i) = str2num(decoded.msg);
%         end
%         set(handles.State,'string',strcat('State:  ',char(mat2string(infoTable))));
%     end
    
function checkLiveResponse(index)
    global liveResponse_UR liveResponse_UL liveResponse_LR liveResponse_LL response requested
    for i = 1:3
        msgIN = liveResponse_UL.getNextMessage(1); %Wait milliseconds
        if length(msgIN) > 0
            decoded = biped_lcm.LiveControlFromTeensy(msgIN.data);
            response(index,decoded.joint+1) = decoded.angle;
            requested(index,decoded.joint+1) = decoded.current;
        end
        msgIN = liveResponse_UR.getNextMessage(1); %Wait milliseconds
        if length(msgIN) > 0
            decoded = biped_lcm.LiveControlFromTeensy(msgIN.data);
            response(index,decoded.joint+4) = decoded.angle;
            requested(index,decoded.joint+4) = decoded.current;
        end
        msgIN = liveResponse_LL.getNextMessage(1); %Wait milliseconds
        if length(msgIN) > 0
            decoded = biped_lcm.LiveControlFromTeensy(msgIN.data);
            response(index,decoded.joint+7) = decoded.angle;
            requested(index,decoded.joint+7) = decoded.current;
        end
        msgIN = liveResponse_LR.getNextMessage(1); %Wait milliseconds
        if length(msgIN) > 0
            decoded = biped_lcm.LiveControlFromTeensy(msgIN.data);
            response(index,decoded.joint+10) = decoded.angle;
            requested(index,decoded.joint+10) = decoded.current;
        end
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
livemsg = biped_lcm.LiveControlAll;
livemsg.num_joints = int8(3);
livemsg.joint_ids = [1 2 3];
livemsg.torque = [0 0 0];
livemsg.angle = [0 0 0];
lc.publish('live_in',livemsg);
livemsg = biped_lcm.LiveControlAll;
livemsg.num_joints = int8(3);
livemsg.joint_ids = [4 5 6];
livemsg.torque = [0 0 0];
livemsg.angle = [0 0 0];
lc.publish('live_in',livemsg);
livemsg = biped_lcm.LiveControlAll;
livemsg.num_joints = int8(3);
livemsg.joint_ids = [7 8 9];
livemsg.torque = [0 0 0];
livemsg.angle = [0 0 0];
lc.publish('live_in',livemsg);
livemsg = biped_lcm.LiveControlAll;
livemsg.num_joints = int8(3);
livemsg.joint_ids = [10 11 12];
livemsg.torque = [0 0 0];
livemsg.angle = [0 0 0];
lc.publish('live_in',livemsg);
for i = 1:4
    msg.command = biped_lcm.commData2Teensy.RUN_STATIC_ALL;
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
joint_angle = get(handles.angle_out,'string');
joint_angle = int16(str2num(joint_angle));
msg.command = biped_lcm.commData2Teensy.RUN_TRAJECTORY;
msg.joint = mod(joint_num-1,3);
teensy = specifyTeensy(joint_num);
lc.publish(strcat(teensy,'cmd_in'), msg)
%Need to send live messages
livemsg = biped_lcm.LiveControlAll;
livemsg.num_joints = int8(1);
livemsg.joint_ids = joint_num;
livemsg.torque = 0;
livemsg.angle = joint_angle;
lc.publish('live_in',livemsg);
set(handles.State,'string','State: Ready to Proceed');
checkLogMsgLoop(handles);
% while true
%   drawnow()
%   stop_state = get(handles.stop, 'Value');
%   if stop_state
%     break;
%   end
%   checkLogMsgLoop(handles);
% end
% checkLogMsgLoop(handles);

function PD_Control(i)
    global lc response requested states_temp torques_temp traj
%     q_d = [0.9; traj.pos(i*10).l_leg_hpy; traj.pos(i*10).l_leg_kny; traj.pos(i*10).l_leg_aky; traj.pos(i*10).r_leg_akx; traj.pos(i*10).r_leg_hpx; traj.pos(i*10).r_leg_hpy; traj.pos(i*10).r_leg_kny; traj.pos(i*10).r_leg_aky; traj.pos(i*10).r_leg_akx];
%     dq_d = [0; traj.vel(i*10).l_leg_hpy; traj.vel(i*10).l_leg_kny; traj.vel(i*10).l_leg_aky; traj.vel(i*10).r_leg_akx; traj.vel(i*10).r_leg_hpx; traj.vel(i*10).r_leg_hpy; traj.vel(i*10).r_leg_kny; traj.vel(i*10).r_leg_aky; traj.vel(i*10).r_leg_akx];
    q_act = [0.9,response(i-1,2:3),response(i-1,7:8),response(i-1,4:6),response(i-1,10:11)]';
    dq_act = q_act - [0.9,response(i-2,2:3),response(i-2,7:8),response(i-2,4:6),response(i-2,10:11)]';
%    q_d = zeros(10,1);
   q_d = [0.3326; -0.6637; 1.2992 ; 
       -0.5496; -0.0368; 
       -0.3623; -0.8477; 1.1945; 
       -0.5977; -0.0014];
   dq_d = zeros(10,1);
    
% %     torques = TorquePD(q_act,q_d,dq_act,dq_d);
    max_angle = -pi/2.2;
    period = 133;
    
    if (mod(floor(i/period),2) == 0)
        q_d = [0,0,0,0,0,0,(max_angle*(mod(i,period)/period)),0,0,0]';
    else
        q_d = [0,0,0,0,0,0,(max_angle*(1-mod(i,period)/period)),0,0,0]';
    end 
    torques = TorquePD([0,response(i-1,2:3),response(i-1,7:8),response(i-1,4:6),response(i-1,10:11)]',q_d,dq_d,dq_d); %We should give specific angles
    gain = 1;
    
%     livemsg = biped_lcm.LiveControlAll;
%     livemsg.num_joints = int8(3);
%     livemsg.joint_ids = [10 11 12];
%     livemsg.torque = [gain*torques(9) torques(10) 0]; 
%     livemsg.angle = [0 0 0];
%     % livemsg.angle = [max(traj.pos(i*10).l_leg_hpx,0)*stronger+0.3 min(traj.pos(i*10).r_leg_hpx,0)*stronger-0.3 -traj.pos(i*10).l_leg_hpy*stronger];
%     lc.publish('live_in',livemsg);
%     livemsg = biped_lcm.LiveControlAll;
%     livemsg.num_joints = int8(3);
%     livemsg.joint_ids = [7 8 9];
%     livemsg.torque = [gain*torques(4) torques(5) 0]; 
%     livemsg.angle = [0 0 0];
%     % livemsg.angle = [max(traj.pos(i*10).l_leg_hpx,0)*stronger+0.3 min(traj.pos(i*10).r_leg_hpx,0)*stronger-0.3 -traj.pos(i*10).l_leg_hpy*stronger];
%     lc.publish('live_in',livemsg);
%     livemsg = biped_lcm.LiveControlAll;
%     livemsg.num_joints = int8(3);
%     livemsg.joint_ids = [1 2 3];
%     livemsg.torque = [1.8 torques(2) torques(3)]; 
%     livemsg.angle = [0 0 0];
%     % livemsg.angle = [max(traj.pos(i*10).l_leg_hpx,0)*stronger+0.3 min(traj.pos(i*10).r_leg_hpx,0)*stronger-0.3 -traj.pos(i*10).l_leg_hpy*stronger];
%     lc.publish('live_in',livemsg);
    livemsg = biped_lcm.LiveControlAll;
    livemsg.num_joints = int8(3);
    livemsg.joint_ids = [4 5 6];
    livemsg.torque = [-2 torques(7) 0]; 
    livemsg.angle = [0 0 0];
    % livemsg.angle = [max(traj.pos(i*10).l_leg_hpx,0)*stronger+0.3 min(traj.pos(i*10).r_leg_hpx,0)*stronger-0.3 -traj.pos(i*10).l_leg_hpy*stronger];
    lc.publish('live_in',livemsg);
    checkLiveResponse(i);
    if i == 2983
        delete(timerfindall);
        save('response.mat','response');
        save('requested.mat','requested');
        save('states_temp.mat','states_temp');
        save('torques_temp.mat','torques_temp');
    end

function gravityComp(i)
    global lc response requested states_temp torques_temp
    max_angle = pi/4;
    period = 133;
    torques = simpleGravComp([0, 0, 0.6010, 0.0001, -0.0028, 0, 0.9,response(i-1,2:3),response(i-1,7:8),response(i-1,4:6),response(i-1,10:11)]'); %We should give specific angles
%     torques = simpleGravComp([0, 0, 0.6010, 0.0001, -0.0028, 0, 1.842, 0.4699, 0.5599,-0.462, 0.017, -0.2420, -0.3750, 0.6383, -0.4812, 0.0198]'); %We should give specific angles

%     torques = [];
%     if (mod(floor(i/period),2) == 0)
%         torques = simpleGravComp([0, 0, 0.6010, 0.0001, -0.0028, 0,response(i-1,1:3),response(i-1,7:8),response(i-1,4),(max_angle*(mod(i,period)/period)),response(i-1,6),response(i-1,10:11)]'); %We should give specific angles
%     else
%         torques = simpleGravComp([0, 0, 0.6010, 0.0001, -0.0028, 0,response(i-1,1:3),response(i-1,7:8),response(i-1,4),(max_angle*(1-mod(i,period)/period)),response(i-1,6),response(i-1,10:11)]'); %We should give specific angles
%     end 

    livemsg = biped_lcm.LiveControlAll;
    livemsg.num_joints = int8(3);
    livemsg.joint_ids = [10 11 12];
    gain = 1.3;
    livemsg.torque = [gain*torques(9) gain*torques(10) 0]; 
    livemsg.angle = [0 0 0];
    % livemsg.angle = [max(traj.pos(i*10).l_leg_hpx,0)*stronger+0.3 min(traj.pos(i*10).r_leg_hpx,0)*stronger-0.3 -traj.pos(i*10).l_leg_hpy*stronger];
    lc.publish('live_in',livemsg);
    livemsg = biped_lcm.LiveControlAll;
    livemsg.num_joints = int8(3);
    livemsg.joint_ids = [7 8 9];
    gain = 1.3;
    livemsg.torque = [gain*torques(4) gain*torques(5) 0]; 
    livemsg.angle = [0 0 0];
    % livemsg.angle = [max(traj.pos(i*10).l_leg_hpx,0)*stronger+0.3 min(traj.pos(i*10).r_leg_hpx,0)*stronger-0.3 -traj.pos(i*10).l_leg_hpy*stronger];
    lc.publish('live_in',livemsg);
    livemsg = biped_lcm.LiveControlAll;
    livemsg.num_joints = int8(3);
    livemsg.joint_ids = [1 2 3];
    gain = 1.3;
    livemsg.torque = [2.5 gain*torques(2) gain*torques(3)]; 
    livemsg.angle = [0 0 0];
    % livemsg.angle = [max(traj.pos(i*10).l_leg_hpx,0)*stronger+0.3 min(traj.pos(i*10).r_leg_hpx,0)*stronger-0.3 -traj.pos(i*10).l_leg_hpy*stronger];
    lc.publish('live_in',livemsg);
    livemsg = biped_lcm.LiveControlAll;
    livemsg.num_joints = int8(3);
    livemsg.joint_ids = [4 5 6];
    gain = 1.3;
    livemsg.torque = [gain*torques(6) gain*torques(7) gain*torques(8)]; %-gain*torques(2) -gain*torques(3)
    livemsg.angle = [0 0 0];
    % livemsg.angle = [max(traj.pos(i*10).l_leg_hpx,0)*stronger+0.3 min(traj.pos(i*10).r_leg_hpx,0)*stronger-0.3 -traj.pos(i*10).l_leg_hpy*stronger];
    lc.publish('live_in',livemsg);
    checkLiveResponse(i);
    if i == 2983
        delete(timerfindall);
        save('response.mat','response');
        save('requested.mat','requested');
        save('states_temp.mat','states_temp');
        save('torques_temp.mat','torques_temp');
    end

% --- Executes on button press in runAll.
function runAll_Callback(hObject, eventdata, handles)
% hObject    handle to runAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global lc msg
set(handles.State,'string','State: Running Trajectory');
joint_num = get(handles.joint_number,'string');
joint_num = int8(str2num(joint_num));
for j = 1:3
for i = 0:3
    livemsg = biped_lcm.LiveControlAll;
    livemsg.num_joints = int8(3);
    livemsg.joint_ids = [i*3+1 i*3+2 i*3+3];
    livemsg.torque = [0 0 0];
    livemsg.angle = [0 0 0];
    lc.publish('live_in',livemsg);
end
end

for i = 1:4
    msg.command = biped_lcm.commData2Teensy.RUN_ALL_TRAJECTORIES;
    teensy = specifyTeensy(i*3);
    lc.publish(strcat(teensy,'cmd_in'), msg);
end
load('/home/gardamerinos/drake-distro/biped/plan_control/traj/12_step_walk_v2.mat')
global traj;
trajLength = 2983;%length(traj.times);
global traj_iteration;
traj_iteration = 2;
t = timer('TimerFcn', @(x,y) mycallback_fcn,...
    'ExecutionMode', 'fixedRate', 'Period', .015, 'TasksToExecute', trajLength);
start(t);

function mycallback_fcn
global traj_iteration;
traj_iteration = traj_iteration+1
% sendLiveMsg(traj_iteration); % use grav_comp instead
% gravityComp(traj_iteration);
PD_Control(traj_iteration);


function sendLiveMsg( i )
global traj lc response requested
stronger = 1;
livemsg = biped_lcm.LiveControlAll;
livemsg.num_joints = int8(3);
livemsg.joint_ids = [1 2 3];
livemsg.torque = [0 0 0];
livemsg.angle = [0 0 0];
% livemsg.angle = [max(traj.pos(i*10).l_leg_hpx,0)*stronger+0.3 min(traj.pos(i*10).r_leg_hpx,0)*stronger-0.3 -traj.pos(i*10).l_leg_hpy*stronger];
lc.publish('live_in',livemsg);
livemsg = biped_lcm.LiveControlAll;
livemsg.num_joints = int8(3);
livemsg.joint_ids = [4 5 6];
livemsg.torque = [-6 0 0];%-i/350
% livemsg.angle = [traj.pos(i*10).r_leg_hpy*stronger traj.pos(i*10).l_leg_kny*stronger traj.pos(i*10).r_leg_kny*stronger];
livemsg.angle = [0 0 0];
lc.publish('live_in',livemsg);
checkLiveResponse(i);
if i == 2982
    delete(timerfindall);
    save('response.mat','response');
    save('requested.mat','requested');
end


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
