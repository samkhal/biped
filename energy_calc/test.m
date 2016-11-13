% consts = {};
% %for i = 2:35
%     constant_range = repmat([-6000;6000],1,2);
%     joints = {'akx'};
%     foo = @(angle,constants)((angle-.3825)).*constants(1);%(polyval(constants(2:end),angle+constants(1)));
%     [const, savings] = find_optimal_energy_system(states, state_frames, torques, torque_frames, foo, constant_range, joints);
%     consts{end+1} = const;
%     min(min(foo(states([14,9],:),const)))
%     hold on;
%     plot(i,savings(1),'*');
% %end

sortedStates = sortrows([states(14,:)', states(30,:)', torques(6,:)']);
edges = min(states(14,:)):1/100*(max(states(14,:))-min(states(14,:))):max(states(14,:));
bins = discretize(sortedStates(:,1),edges);
keys = 1:100;
values = cell(1,100);
i=0;
j=1;
while i < size(bins,1)
i = i+1;
while(i < size(bins,1) && bins(i) == j)
    values{j} = [values{j} sortedStates(i,:)'];
    i = i +1;
end
j=j+1;
end
mapObj = containers.Map(keys,values);
A = mapObj(100);
plot(A(1,:),(A(3,:)),'*')
xlabel('angle rad');
ylabel('torque Nm');
