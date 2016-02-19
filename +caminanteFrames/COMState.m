classdef COMState < SingletonCoordinateFrame
  methods
    function obj=COMState(dof)
      if nargin < 1
        dof = 3;
      end
      obj = obj@SingletonCoordinateFrame('caminanteFrames.COMState',2*dof,'x');
    end
  end
end
