classdef Standing < caminanteParams.Base
  methods
    function obj = Standing(r)
      typecheck(r, 'Caminante');
      obj = obj@caminanteParams.Base(r);
    end
  end
end


