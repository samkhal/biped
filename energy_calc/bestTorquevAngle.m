%% Ari Goodman
%% 11/16/2016

%% THIS FILE IS STILL UNDER CONSTRUCTION

%% for a given trajectory, finds the optimal torques at each angle to minimizes energy consumption for a single joint.

%% inputs
% map  = (bin, {[position-rad speed-rad/s torque-Nm]...})

%% outputs
% best_torques matrix of best torques-Nm

function best_torques = bestTorquevAngle(joint, traj)
    tic
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
        energyFun = @(input_torque)(sum(electrical_power([M,R],tempBin(2,:),tempBin(3,:)-input_torque,0)));
        best_torques(i) = fminbnd(energyFun, min(tempBin(3,:)),max(tempBin(3,:)));       
    end
    %TODO ADD A PLOT
    
end