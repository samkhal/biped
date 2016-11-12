consts = {};
for i = 2:35
    constant_range = repmat([-6000;6000],1,i);
    joints = {'kny'};
    foo = @(angle,constants)(polyval(constants(2:end),angle+constants(1)));
    [const, savings] = find_optimal_energy_system(states, state_frames, torques, torque_frames, foo, constant_range, joints);
    consts{end+1} = const;
    min(min(foo(states([14,9],:),const)))
    hold on;
    plot(i,savings(1),'*');
end
