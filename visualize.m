% Visualize biped in Drake

m = RigidBodyManipulator('urdf/Legs.urdf');
v = m.constructVisualizer();
%v.inspector() % This line currently crashes matlab