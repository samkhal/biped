% consts = {};
% %for i = 2:35
%     joints = {'l_leg_kny'};
%     constant_range = [-min([traj.pos.(joints{1})]) -6000;pi/2-max([traj.pos.(joints{1})]) 6000]
%     foo = @(angle,constants)((angle+constants(1))).*constants(2);%(polyval(constants(2:end),angle+constants(1)));
%     [const, savings] = find_optimal_energy_system(traj, foo, constant_range, joints);
%     consts{end+1} = const;
%     %min(min(foo(states([14,9],:),const)))
%     %hold on;
%     %plot(i,savings(1),'*');
% %end

% TODO TODO TODO
joint = 'r_leg_kny'
sortedStates = sortrows([[traj.pos.(joint)]', [traj.vel.(joint)]', [traj.torque.(joint)]']);
edges = min([traj.pos.(joint)]):1/100*(max([traj.pos.(joint)])-min([traj.pos.(joint)])):max([traj.pos.(joint)]);
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

% data = zeros(size(mapObj,1),1);
% for i = 1:size(mapObj,1)
%     tempBin = mapObj(i);
%     data(i) = mean(tempBin(3,:));
% end
% 
% plot(1:100,data,'*')
% xlabel('bin number');
% ylabel('torque Nm');



