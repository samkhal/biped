function out = matlab_serial_example(functionality,teensy,link)

global biped;
biped = Biped(1:13);
% load bipedData biped;
% biped = loadobj(bipedData);
global data;
global out;
out = [];
data = -1;
valuesPerSample = 1;
s = serialSetup();

% for i = s
%     fopen(i);
% end
cleanupObj = onCleanup(@() delete(instrfindall));
    
% Each number is a position for 10 milliseconds
% Every 1 sec of trajectory for every 100 values
dataOut = 0;
if link == 1 
    dataOut = 500:-0.5:300;
    dataOut = [dataOut 300:0.5:500]; 
elseif link == 2
    dataOut = 600:-0.5:400;
    dataOut = [dataOut 400:0.5:600];
elseif link == 3
    dataOut = 400:0.5:600;
    dataOut = [dataOut 600:-0.5:400];
end
dataOut = floor(dataOut);
total_trajectory_seconds = numel(dataOut)/100;

%State machine
if functionality == 0   
    runTrajectory(s,link);
elseif functionality == 1
    sendTrajectory(s,dataOut,link);
elseif functionality == 2
    startCalibration(s,link);
elseif functionality == 3
    stopCalibration(s,link)
elseif functionality == 4
    runStaticControl(s,link);
elseif functionality == 5
    stopStaticControl(s);
elseif functionality == 6
    idRequest(s);
end
    
if (data == 0)
    disp 'ready to proceed';
elseif(data == 2)
    disp('Something went wrong');
elseif(data == 3)
    disp 'Trajectory run';
end
% for i = s
%     fclose(i);
% end
% bipedData = saveobj(biped);
save bipedData.mat biped;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%FUNCTIONS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function s = serialSetup()
    function receive_data(serial_obj,event)
        global out;
        global data;
        data = fread(serial_obj, valuesPerSample, 'uint16');
        disp(data);
        out= [out data];
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
    end
end

function out = linkToSerial(link)
    global biped;
    out = biped.IDSerialMap(link);
end

function send_data(s,dataOut)
    for i = dataOut;
        fwrite(s,i,'int16');
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
    while data~=1
        pause(0.00001);
    end
    disp('Running Trajectory');
    % tranceive data while there are still data
end
function sendTrajectory (s,dataOut,link)
    global data;
    global biped;
    send_init_bytes(s(biped.IDSerialMap(link)),dataOut,link)
    while data ~= numel(dataOut)
        pause(0.01);
    end
    send_data(s(biped.IDSerialMap(link)),dataOut)
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
        pause(0.1);
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
function idRequest(s)
    global data;
    global biped;
    global out;
    ID = [];
    SERIAL = [];
    for i = 1:length(s);
        data = -1;
        fopen(s(i));
        fwrite(s(i),14,'uint8');
        for j = 1:9
            fwrite(s(i),1,'uint8');
        end
        while data ~= 0
            pause(0.000000001);
        end
        fclose(s(i));
        SERIAL = [SERIAL i i i];
        ID = [ID out(end-3) out(end-2) out(end-1)];
    end
    biped.IDSerialMap = containers.Map(ID,SERIAL);
    disp('Got IDs');
end