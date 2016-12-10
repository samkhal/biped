classdef CaminanteCoordinates < SingletonCoordinateFrame
  % caminante q
  methods
    function obj=CaminanteCoordinates(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      nq = r.getNumPositions();
      coords = r.getStateFrame().getCoordinateNames();
      obj = obj@SingletonCoordinateFrame('caminanteFrames.CaminanteCoordinates',nq,'x',coords(1:nq));
    end
  end
end
