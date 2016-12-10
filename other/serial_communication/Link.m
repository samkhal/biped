classdef Link
    properties
        % with perspective as we are the robot :P
        linkMap = containers.Map([1:12] , ...
        {Joints.l_leg_toe, Joints.l_leg_akx, Joints.l_leg_aky, Joints.l_leg_kny,...
        Joints.l_leg_hpy, Joints.l_leg_hpx, Joints.r_leg_hpx, Joints.r_leg_hpy, ...
        Joints.r_leg_kny, Joints.r_leg_aky, Joints.r_leg_akx, Joints.r_leg_toe})
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
