function playbackTrajectory(traj)

r = Caminante('urdf/caminante_minimal_gen.urdf',traj.robot_options);                    
r = compile(r);

v = r.constructVisualizer;

v.playback(traj.ytraj, struct('slider', true));