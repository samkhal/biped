classdef Biped
    properties
        links = [];
        status;
        statusMap = containers.Map([1:3],{'Ready','Not Ready','Nothing Connected'});
        IDMap = containers.Map('ID','int','LinkName','string')
        IDSerialMap;
    end
    methods
        function obj = Biped(ListOfNumbers)
            for i = ListOfNumbers
                obj.links = [obj.links Link(i)];
            end
        end
        function out = getStatus(obj)
            total = length(obj.links);
            counter = 0;
            for i = obj.links
                if i.status == 1
                    counter = counter+1;
                end
            end
            if counter == 0
                out = obj.statusMap(3);
            elseif counter == total
                out = obj.statusMap(1);
            else
                out = obj.statusMap(2);
            end
        end
        function set_ID(obj,Linkname,IDs)
            obj.IDMap = containers.Map(IDs,Linkname);
        end
    end
end