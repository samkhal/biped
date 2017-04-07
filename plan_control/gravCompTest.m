%gravCompTest
options.floating = true;
options.dt = 0.001;
options.terrain = RigidBodyFlatTerrain;
r = Caminante('urdf/caminante_minimal.urdf',options);
r = compile(r);
c = [];
t= -3:0.05:0;
for i = 0:-0.05:-3
[H C B dH dC dB] = r.manipulatorDynamics([0, 0, 0.6010, 0.0001, -0.0028, 0, -0.0143, 0.0009, 0.0009, 0.0009, -0.0091, -0.0144, 0.0009, 0.0009, 0.0009, 0.0089]', zeros(r.getNumVelocities,1));
c = [c,C(8)];
end
f = figure();
plot (t,c,'DisplayName','torque from gravity');
name = 'Left Hip Y';
title(name);
xlabel('radians');
ylabel('torque - Nm');
legend('show','Location','southeast');

saveas(f,strcat(name,'.jpg'));