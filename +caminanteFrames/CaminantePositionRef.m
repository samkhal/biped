classdef CaminantePositionRef < SingletonCoordinateFrame
  % caminante position reference input frame
  methods
    function obj=CaminantePositionRef(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      input_names = r.getInputFrame().getCoordinateNames();
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      obj = obj@SingletonCoordinateFrame('caminanteFrames.CaminantePositionRef',r.getNumInputs(),'x',input_names)
    end
  end
end
