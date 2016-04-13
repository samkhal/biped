classdef TrajectoryPlanner < DrakeSystem
    properties
        x0;
        trajectory;
    end
    methods
        function obj = TrajectoryPlanner(sys,x0)
            obj@DrakeSystem(0, ... 0 cts time state vars
                            0, ... 0 dsc time state vars
                            0, ... 0 inputs
                            sys.getNumInputs, ... number of outputs
                            false, ... no direct feedthrough
                            true); ... time-invariant
            obj = obj.setOutputFrame(sys.getInputFrame);
            obj.x0 = x0((1:12) + 6);
            obj.trajectory = generateWalkingTrajectory(); 
        end
        
        function x0 = getInitialState(obj)
            x0 = obj.x0;
        end
        
        function  y = output(obj,t,x,u) %y = output(~,~,~,~) 
            %{
              1  Biped__Body-Left_Hip
              2  Biped__Left_Hip-Top_Left_Leg
              3  Biped__Top_Left_Leg-Bottom_Left_Leg
              4  Biped__Bottom_Left_Leg-Left_Ankle
              5  Biped__Left_Ankle-Left_Foot
              6  Biped__Left_Foot-Left_Toe
              7  Biped__Body-Right_Hip
              8  Biped__Right_Hip-Top_Right_Leg
              9  Biped__Top_Right_Leg-Bottom_Right_Leg
              10 Biped__Bottom_Right_Leg-Right_Ankle
              11 Biped__Right_Ankle-Right_Foot
              12 Biped__Right_Foot-Right_Toe
            %}
%             y = zeros(12,1);
            %y = [0, -0.1, 0.3, -0.4, 0, 0, ...
            %     0, -0.1, 0.3, -0.4, 0, 0 ];
            
            %tempTraj = generateWalkingTrajectory();
            ytemp = obj.trajectory.eval(t);
            y = ytemp(1:12+6)';
            %y = [0*t, -0.1, 0.3, -0.4, 0, 0, ...
            %     0, -0.1, 0.3, -0.4, 0, 0 ]
        end
        
    end
end