% Sam Khalandovsky
% Run torque function optimization across various orders of polynomial

consts = {};
for i = 2:35
    joints = {'r_leg_kny'};
    const_range = repmat([-6000;6000],1,i-1);
    constant_range = horzcat([-min([traj.pos.(joints{1})]);pi/2-max([traj.pos.(joints{1})])], const_range);
    foo = @(angle,constants) (polyval(constants(2:end),angle+constants(1)));
    [const, savings] = find_optimal_energy_system(traj, foo, constant_range, joints);
    consts{end+1} = const;

    hold on;
    plot(i,savings(1),'*');
end