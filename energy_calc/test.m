constant_range = [-6000 -.7; 6000 .2];
joints = {'kny'};
foo = @(angle,constants)((angle+constants(1))*constants(2));
[const savings] = find_optimal_energy_system(states, state_frames, torques, torque_frames, foo, constant_range, joints);