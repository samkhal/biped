classdef Recovery < caminanteParams.Walking
  methods
    function obj = Recovery(r)
      obj = obj@caminanteParams.Walking(r);
      obj.body_motion(r.foot_body_id.right).Kp = 10 * [100; 100; 100; 150; 150; 150];
      obj.body_motion(r.foot_body_id.left).Kp = 10 * [100; 100; 100; 150; 150; 150];
      obj = obj.updateKd();
    end
  end
end