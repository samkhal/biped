classdef Link
    properties
        % with perspective as we are the robot :P
        linkMap = containers.Map([1:13] , ...
        {'L_HIP','R_HIP','L_UPPER_LEG','R_UPPER_LEG','L_KNEE','R_KNEE','L_ANKLE','R_ANKLE','L_FOOT','R_FOOT','L_TOE','R_TOE','IMU'})
        statusMap = containers.Map([1:3],{'CONNECTED','DISCONNECTED','RUNNING'});
        name;           %Name of the link
        numberOfLink;   %number of link from the overall robot (we get it from the Teensy) 
        status;         %connected - disconnected - running
    end
    methods
        function obj = Link(i)
            obj.numberOfLink = i;
            obj.name = obj.linkMap(i);
        end
        function out = getStatus(obj)
            out = obj.statusMap(obj.status);
        end
    end
end
