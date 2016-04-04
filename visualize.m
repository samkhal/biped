% Visualize biped in Drake

options.floating = true;
m = RigidBodyManipulator('urdf/Legs.urdf',options);
%m = RigidBodyManipulator('~/drake-distro/drake/examples/Atlas/urdf/atlas_minimal_contact.urdf',options);
v = m.constructVisualizer();
%v.inspector(zeros(m.getNumStates,1));
v.inspector(zeros(24+12,1)) % Init inspector with all joints at 0

% Note: body rolls and pitches wrong, likely frame is in the wrong place