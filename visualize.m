% Visualize biped in Drake

m = RigidBodyManipulator('urdf/Legs.urdf');
v = m.constructVisualizer();
v.inspector(zeros(24,1)) % Init inspector with all joints at 0