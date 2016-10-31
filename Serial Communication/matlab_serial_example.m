function data = matlab_serial_example
data = 0;
bytesPerValue = 2;
valuesPerSample = 1;

s = serial('/dev/ttyACM2');
s.Baudrate = 115200;
s.BytesAvailableFcnCount = bytesPerValue * valuesPerSample;
s.BytesAvailableFcnMode = 'byte';
s.BytesAvailableFcn = @receive_data;

function receive_data(serial_obj,event)
    data = fread(serial_obj, valuesPerSample, 'uint16');
    disp(data);
end

fopen(s);
cleanupObj = onCleanup(@() delete(instrfindall));
dataOut = 500:4000;%ones(1,500)*20;
runTrajectory(s);
% send_init_bytes(s,dataOut)
% while data ~= numel(dataOut)
%     pause(0.01);
% end
% send_data(s,dataOut)
% pause(0.01);
% while data == numel(dataOut)
%     pause(0.01);
% end
if (data == 0)
    disp 'ready to proceed';
elseif(data == 2)
    disp 'Something went wrong';
elseif(data == 3)
    disp 'Trajectory running';
end
fclose(s);
end

function send_data(s,dataOut)
    for i = dataOut;
        fwrite(s,i,'uint16');
    end
end

%Initialization bytes with the sum of the data to transmit:
function send_init_bytes(s,dataOut)
    summation = uint32(sum(dataOut));
    fwrite(s,1,'uint8');
    pause(0.001);
    fwrite(s,numel(dataOut),'uint32');
    pause(0.001);
    fwrite(s,summation,'uint32');
end
function runTrajectory(s)
    for i = 1:9
        fwrite(s,256,'uint8');
    end
    pause(0.1);
    %When trajectory is complete receive data
end