% Visualize biped in Drake

options.floating = true;
%m = RigidBodyManipulator('urdf/Legs.urdf',options);
%m = Caminante('~/drake-distro/drake/examples/Caminante/urdf/caminante_minimal_contact.urdf',options);
m = Caminante('urdf/caminante_experimental.urdf',options);
%m = Caminante('urdf/caminante_minimal_contact.urdf',options);
v = m.constructVisualizer();
%v.inspector(zeros(m.getNumStates,1));
x0=zeros(m.getNumStates,1);
%x0 = load('data/caminante_fp.mat','xstar');
x0 = load('data/caminante_exp_fp.mat','xstar');
x0 = x0.xstar;
%x0(1:18)=q0;
v.inspector(x0) % Init inspector with all joints at 0

% Note: body rolls and pitches wrong, likely frame is in the wrong place