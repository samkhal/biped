constant_range = [-6000 -6000 -6000 -6000; 6000 6000 6000 6000];
joints = {'kny'};
foo = @(angle,constants)((angle.^3)*constants(4)+abs(angle.^2)*constants(3)+(angle)*constants(2)+constants(1));
[const, savings] = find_optimal_energy_system(states, state_frames, torques, torque_frames, foo, constant_range, joints);
min(min(foo(states([14,9],:),const)))
