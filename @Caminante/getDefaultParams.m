function param_sets = getDefaultParams(r)
% NOTEST
  param_sets = struct('walking', caminanteParams.Walking(r),...
                      'standing', caminanteParams.Standing(r),...
                      'position_control', caminanteParams.PositionControl(r),...
                      'recovery', caminanteParams.Recovery(r),...
                      'manip',caminanteParams.Manip(r));
end
