classdef TorqueController < DrakeSystem
    methods
        function obj = TorqueController()
            obj@DrakeSystem(0, ... 0 cts time state vars
                            0, ... 0 dsc time state vars
                            0, ... 0 inputs
                            12, ... 12 outputs
                            false, ... no direct feedthrough
                            true); ... time-invariant
        end
        
        function x0 = getInitialState(obj)
            x0 = zeros(1,obj.getNumOutputs);
        end
        
        function y = output(obj,t,x,u)
            % play around with this function to make the legs do weird shit
            y = zeros(obj.getNumOutputs);
            if (t<.2)    
                y([9,3]) = 20;
            elseif (t<.25)
                y([9,3]) = -50;
            else
                y([9,3]) = 0;
            end
        end
        
    end
end