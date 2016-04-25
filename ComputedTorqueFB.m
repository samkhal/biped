classdef ComputedTorqueFB < DrakeSystem
    properties
        Kp;
        Kd;
        r;
    end
    
    methods
        function obj = ComputedTorqueFB(r,Kp,Kd)
            obj@DrakeSystem(0, ... 0 cts time state vars
                            0, ... 0 dsc time state vars
                            r.getNumOutputs, ... 0 inputs
                            r.getNumInputs, ... 12 outputs
                            false, ... no direct feedthrough
                            true); ... time-invariant
            obj = obj.setOutputFrame(r.getInputFrame);
            obj = obj.setInputFrame(r.getOutputFrame);
            
            obj.Kp = Kp;
            obj.Kd = Kd;
            obj.r = r;
        end
        
%         function x0 = getInitialState(obj)
%             x0 = zeros(obj.getNumOutputs,1);
%         end
        
        function y = output(obj,t,~,u) % (obj,t,x,u)
            q = u((1:12) + 6);
            v = u((1:12) + 24);
            
            [M,C,B] = obj.r.manipulatorDynamics(q,v);
            
            M = M(7:18,7:18);
            C = C(7:18);
            B = B(7:18,:);
            
            y = B\(M*obj.Kp*q + M*obj.Kd*v);% + C);
        end
        
    end
end