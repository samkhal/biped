classdef CaminanteForceTorque < SingletonCoordinateFrame
  
  methods
    function obj=CaminanteForceTorque()

%       coordinates = {'l_foot_fz','l_foot_tx','l_foot_ty',...
%                 'r_foot_fz','r_foot_tx','r_foot_ty'};

      obj = obj@SingletonCoordinateFrame('caminanteFrames.CaminanteForceTorque',length(coordinates),'f',coordinates);              
    end
  end
end
