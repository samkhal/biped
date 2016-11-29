function out = matlab_serial_example(functionality,ID)

global biped;
% biped = Biped(1:12);
load bipedData biped;
global data;
global traj;
global out;
out = cell(1,4);
data = -1;
valuesPerSample = 1;
s = serialSetup();

for i = s
    fopen(i);
end
cleanupObj = onCleanup(@() delete(instrfindall));
    
% Each number is a position for 10 milliseconds
% Every 1 sec of trajectory for every 100 values
% dataOut = 0;
% if ID == 1 
%     dataOut = 500:-0.5:300;
%     dataOut = [dataOut 300:0.5:500]; 
% elseif ID == 2
%     dataOut = 600:-0.5:400;
%     dataOut = [dataOut 400:0.5:600];
% elseif ID == 3
%     dataOut = 400:0.5:600;
%     dataOut = [dataOut 600:-0.5:400];
% end
% dataOut = floor(dataOut);
% total_trajectory_seconds = numel(dataOut)/100;

%State machine
if functionality == 0   
    runTrajectory(s(IDToSerial(ID)),IDToLink(ID));
elseif functionality == 1
    traj = load ('five_step_walk_v3.mat');
    dataOut = calcDataOut(ID);
    sendTrajectory(s(IDToSerial(ID)),dataOut,ID);
elseif functionality == 2
    startCalibration(s(IDToSerial(ID)),IDToLink(ID));
elseif functionality == 3
    stopCalibration(s(IDToSerial(ID)),IDToLink(ID))
elseif functionality == 4
    runStaticControl(s(IDToSerial(ID)),IDToLink(ID));
elseif functionality == 5
    stopStaticControl(s(IDToSerial(ID)));
elseif functionality == 6
    idRequest(s);
elseif functionality == 7
    runAllTrajectories(s);
elseif functionality == 8
    runAllStatic(s);
elseif functionality == 9
    stopAllStatic(s);
end
    
if (data == 0)
    disp 'ready to proceed';
elseif(data == 2)
    disp('Something went wrong');
elseif(data == 3)
    disp 'Trajectory run';
end
for i = s
    fclose(i);
end
% bipedData = saveobj(biped);
save bipedData.mat biped;
if functionality == 3
    out = out(IDToSerial(ID));
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%FUNCTIONS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function s = serialSetup()
    function receive_data(serial_obj,event)
        global out;
        global data;
        data = fread(serial_obj, valuesPerSample, 'uint16');
        i = serial_obj.Name;
        i = str2num(i);
        disp(data);
        out(i) = {[cell2mat(out(i));data]};
    end
    serialInfo = instrhwinfo('serial');
    s = [];
    total = length(serialInfo.SerialPorts);
    for i = 1:total;
        s1 = serial(char(serialInfo.SerialPorts(i)));
        s = [s;s1];
    end
    disp(strcat(num2str(length(s)),' ports available'));
    bytesPerValue = 2;
    valuesPerSample = 1;
    for i = 1:length(s)
        s(i).Baudrate = 115200;
        s(i).BytesAvailableFcnCount = bytesPerValue * valuesPerSample;
        s(i).BytesAvailableFcnMode = 'byte';
        s(i).BytesAvailableFcn = @receive_data;
        s(i).Name = num2str(i);
    end
end

function dataOut = calcDataOut(ID)
    global traj;
    dataOut = zeros(floor(length(traj.traj.pos)/10)+1,1);
    count = 1;
    for i = 1:10:length(traj.traj.pos)
        if ID == 1 
            dataOut(count) = traj.traj.pos(i).l_leg_toe * 1000;
        elseif ID == 2 
            dataOut(count) = traj.traj.pos(i).l_leg_akx * 1000;
        elseif ID == 3 
            dataOut(count) = traj.traj.pos(i).l_leg_aky * 1000;
        elseif ID == 4 
            dataOut(count) = traj.traj.pos(i).l_leg_kny * 1000;
        elseif ID == 5 
            dataOut(count) = traj.traj.pos(i).l_leg_hpy * 1000;
        elseif ID == 6 
            dataOut(count) = traj.traj.pos(i).l_leg_hpx * 1000;
        elseif ID == 7 
            dataOut(count) = traj.traj.pos(i).r_leg_hpx * 1000;
        elseif ID == 8 
            dataOut(count) = traj.traj.pos(i).r_leg_hpy * 1000;
        elseif ID == 9 
            dataOut(count) = traj.traj.pos(i).r_leg_kny * 1000;
        elseif ID == 10 
            dataOut(count) = traj.traj.pos(i).r_leg_aky * 1000;
        elseif ID == 11 
            dataOut(count) = traj.traj.pos(i).r_leg_akx * 1000;
        elseif ID == 12 
            dataOut(count) = traj.traj.pos(i).r_leg_toe * 1000;
        end
        count = count+1;
    end
    dataOut = floor(dataOut);
end

function out = IDToSerial(ID)
    global biped;
    out = biped.IDSerialMap(ID);
end

function out = IDToLink(ID) %ID(1-12) to Link(1-3)
    out = mod(ID-1,3)+1;
end

function send_data(s,dataOut)
    for i = 1:length(dataOut);
        fwrite(s,dataOut(i),'int16');
    end
end

%Initialization bytes with the sum of the data to transmit:
function send_init_bytes(s,dataOut,link)
    summation = uint32(sum(dataOut));
    fwrite(s,1,'uint8');
    fwrite(s,link,'uint8');
    pause(0.001);
    fwrite(s,numel(dataOut),'uint32');
    pause(0.001);
    fwrite(s,summation,'uint32');
end
function runTrajLive(s,link)
    global data;
    fwrite(s,254,'uint8');
    for i = 1:9
        fwrite(s,link,'uint8');
    end
    while data~=1
        pause(0.00001);
    end
    disp('Running Trajectory');
end
function runTrajectory(s,link)
    global data;
    fwrite(s,255,'uint8');
    for i = 1:9
        fwrite(s,link,'uint8');
    end
    disp('Running Trajectory');
    while data~=3
        pause(0.1);
    end
    % tranceive data while there are still data
end
function runAllTrajectories(s)
    global data;
    for x = 1:length(s)
        fwrite(s(x),255,'uint8');
    end
    for i = 1:9
        for x = 1:length(s)
            fwrite(s(x),4,'uint8');
        end
    end
    disp('Running Trajectory');
    while data~=3
        pause(0.01);
    end
    % tranceive data while there are still data
end
function runAllStatic(s)
    global data;
    for x = 1:length(s)
        fwrite(s(x),12,'uint8');
    end
    for i = 1:9
        for x = 1:length(s)
            fwrite(s(x),1,'uint8');
        end
    end
    disp('Running Static');
    while data~=0
        pause(0.00001);
    end
    % tranceive data while there are still data
end
function sendTrajectory (s,dataOut,ID)
    global data;
    global biped;
    send_init_bytes(s,dataOut,IDToLink(ID))
    while data ~= numel(dataOut)
        pause(0.01);
    end
    send_data(s,dataOut)
    pause(0.01);
    while data == numel(dataOut)
        pause(0.01);
    end
end
function startCalibration(s,link)
    global data;
    fwrite(s,10,'uint8');
    for i = 1:9
        fwrite(s,link,'uint8');
    end
    while data~= 0
        pause(0.1);
    end
    disp('Calibration Started');
end
function stopCalibration(s, link)
    global data;
    fwrite(s,11,'uint8');
    for i = 1:9
        fwrite(s,link,'uint8');
    end
    while data~= 0
        pause(0.3);
    end
    disp('Min & Max pot values above');
    disp('If 666, then Out Of Range Issue');
end
function runStaticControl(s,links)
    global data;
    fwrite(s,12,'uint8');
    for i = 1:9
        fwrite(s,links,'uint8');
    end
    while data ~= 0
        pause(0.00001);
    end
    disp('Static Control Initialized');
end
function stopStaticControl(s)
    global data;
    fwrite(s,13,'uint8');
    while data ~= 0
        pause(0.1);
    end
    disp('Static Control Stoped');
end
function stopAllStatic(s)
    global data;
    for x = 1:length(s)
        fwrite(s(x),13,'uint8');
    end
    while data ~= 0
        pause(0.5);
    end
    disp('Static Control Stoped');
end
function idRequest(s)
    global data;
    global biped;
    global out;
    ID = [];
    SERIAL = [];
    for i = 1:length(s);
        data = -1;
%         fopen(s(i));
        fwrite(s(i),15,'uint8');
        for j = 1:9
            fwrite(s(i),1,'uint8');
        end
        while data ~= 0
            pause(0.000000001);
        end
%         fclose(s(i));
        SERIAL = [SERIAL i i i];
        ID = [ID out{i}(end-3) out{i}(end-2) out{i}(end-1)];
    end
    biped.IDSerialMap = containers.Map(ID,SERIAL);
    disp('Got IDs');
end