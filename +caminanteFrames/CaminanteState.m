classdef CaminanteState < SingletonCoordinateFrame
  
  methods
    function obj=CaminanteState(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      manipStateFrame = r.getManipulator().getStateFrame();
      % sanity check for stateless hands
      if (r.hand_left > 0 || r.hand_right > 0 || r.external_force ~= 0)
        manipStateFrame = manipStateFrame.getFrameByNum(1);
      end
      coordinates = manipStateFrame.getCoordinateNames();
      obj = obj@SingletonCoordinateFrame('caminanteFrames.CaminanteState',length(coordinates),'x',coordinates);
      positionFrame = r.getManipulator().getPositionFrame();
      if getNumFrames(positionFrame)==1 && isempty(findTransform(obj,positionFrame))
        obj.addProjectionTransformByCoordinateNames(positionFrame);
      end
    end
  end
end
