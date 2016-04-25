classdef ComputedTorqueFF < DrakeSystem
    properties
        Kp;
        Kd;
        r;
    end
    
    methods
        function obj = ComputedTorqueFF(r,Kp,Kd)
            obj@DrakeSystem(0, ... 0 cts time state vars
                            0, ... 0 dsc time state vars
                            r.getNumInputs, ... 0 inputs
                            r.getNumInputs, ... 12 outputs
                            false, ... no direct feedthrough
                            true); ... time-invariant
            obj = obj.setOutputFrame(r.getInputFrame);
            
            obj.Kp = Kp;
            obj.Kd = Kd;
            obj.r = r;
        end
        
%         function x0 = getInitialState(obj)
%             x0 = zeros(obj.getNumOutputs,1);
%         end
        
        function y = output(obj,~,~,u) % (obj,t,x,u)
            qd = u.';
            vd = zeros(size(qd));
            
            [M,~,B] = obj.r.manipulatorDynamics(qd,vd);
            
            B = B(7:18,:);
            M = M(7:18,7:18);
            
            % assume desired acceleration is 0
            y = B\(M*obj.Kp*qd + M*obj.Kd*vd);
        end
        
    end
end