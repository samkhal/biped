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
            y = ones(obj.getNumOutputs)*-10;
        end
        
    end
end