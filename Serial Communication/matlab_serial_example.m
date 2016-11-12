function out = matlab_serial_example(functionality,teensy,link)
% functionality = 4;

global data 
out = [];
data = -1;
bytesPerValue = 2;
valuesPerSample = 1;

s = serial('/dev/ttyACM0');
s.Baudrate = 115200;
s.BytesAvailableFcnCount = bytesPerValue * valuesPerSample;
s.BytesAvailableFcnMode = 'byte';
s.BytesAvailableFcn = @receive_data;

function receive_data(serial_obj,event)
    data = fread(serial_obj, valuesPerSample, 'uint16');
    if data == 666
        disp(data);
    else
        disp(data);
    end
    out= [out data];
end

fopen(s);
cleanupObj = onCleanup(@() delete(instrfindall));
    
% Each number is a position for 10 milliseconds
% Every 1 sec of trajectory for every 100 values
dataOut = 1:10;
% dataOut = 400:0.1:500;
% dataOut = [dataOut 500:-0.1:400];
% dataOut = floor(dataOut);
total_trajectory_seconds = numel(dataOut)/100;

%State machine
if functionality == 0   
    runTrajectory(s,link);
elseif functionality == 1
    sendTrajectory(s,dataOut,link);
elseif functionality == 2
    startCalibration(s,link);
elseif functionality == 3
    stopCalibration(s)
elseif functionality == 4
    runStaticControl(s,link);
elseif functionality == 5
    stopStaticControl(s);
end
    
if (data == 0)
    disp 'ready to proceed';
elseif(data == 2)
    disp('Something went wrong');
elseif(data == 3)
    disp 'Trajectory run';
end
fclose(s);
end

function send_data(s,dataOut)
    for i = dataOut;
        fwrite(s,i,'uint16');
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
function runTrajectory(s,link)
    global data;
    fwrite(s,255,'uint8');
    for i = 1:9
        fwrite(s,link,'uint8');
    end
    while data~=3
        pause(0.0001);
    end
end
function sendTrajectory (s,dataOut,link)
    global data;
    send_init_bytes(s,dataOut,link)
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
function stopCalibration(s)
    global data;
    fwrite(s,11,'uint8');
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