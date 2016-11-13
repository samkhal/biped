consts = {};
%for i = 2:35
    constant_range = repmat([-6000;6000],1,2);
    joints = {'akx'};
    foo = @(angle,constants)((angle-.3825)).*constants(1);%(polyval(constants(2:end),angle+constants(1)));
    [const, savings] = find_optimal_energy_system(states, state_frames, torques, torque_frames, foo, constant_range, joints);
    consts{end+1} = const;
    min(min(foo(states([14,9],:),const)))
    hold on;
    plot(i,savings(1),'*');
%end
