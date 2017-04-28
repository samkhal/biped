%% Ari Goodman
%% 11/16/2016

%% for a given trajectory, finds the optimal torques at each angle to minimizes energy consumption for a single joint.

%% inputs
% joint: name of joint being optimized
% traj, trajectory input

%% outputs
% best_torques matrix of best torques-Nm separated into a positive and
% negative function

function best_torques = bestTorquevAngle(joint, traj)
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
    map = containers.Map(keys,values);
    M = 14;   %torque constnat
    R = .527; %resistance
    % for each bin, find the supplemented torque which minimizes total
    % energy
    best_torques = zeros(size(map,1),1);
    for i = 1:size(map,1)
        tempBin = map(i);
        energyFun = @(input_torque)(sum(electrical_power([M,R],tempBin(2,:),abs(tempBin(3,:)-input_torque),0)));
        best_torques(i,1) = fminbnd(energyFun, min(0, min(tempBin(3,:))),min(0,max(tempBin(3,:))));      
        best_torques(i,2) = fminbnd(energyFun, max(0, min(tempBin(3,:))),max(0,max(tempBin(3,:))));       
    end
    figure;
    hold on;
    plot(edges(1:(end-1)),best_torques(:,1),'r', 'linewidth', 8)
    plot(edges(1:(end-1)),best_torques(:,2),'b', 'linewidth', 8)
    xlabel('angle rad');
    ylabel('torque Nm');
    title(strcat(joint, 's best input torque v angle'),'Interpreter','none');
    legend( 'negative','positive');
    set(gca,'fontsize',22);
end