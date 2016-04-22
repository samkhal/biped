% Visualize biped in Drake

options.floating = true;
m = RigidBodyManipulator('urdf/Legs.urdf',options);
%m = RigidBodyManipulator('~/drake-distro/drake/examples/Atlas/urdf/atlas_minimal_contact.urdf',options);
v = m.constructVisualizer();
%v.inspector(zeros(m.getNumStates,1));
x0=zeros(m.getNumStates,1);
%x0(1:18)=q0;
v.inspector(x0) % Init inspector with all joints at 0

% Note: body rolls and pitches wrong, likely frame is in the wrong place