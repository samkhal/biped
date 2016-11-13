%% Ari Goodman
%% 11/13/2016

%% THIS FILE IS STILL UNDER CONSTRUCTION

%% for a given trajectory, finds the optimal torques at each angle to minimizes energy consumption for a single joint.

%% inputs
% map  = (bin, {[position-rad speed-rad/s torque-Nm]...})

%% outputs
% best_torques matrix of best torques-Nm

function best_torques = bestTorquevAngle(map)
    M = 14;   %torque constnat
    R = .527; %resistance
    % for each bin, find the supplemented torque which minimizes total
    % energy
    for i = 1:size(map,1)
        tempBin = map(i);
        energyFun = @(input_torque)(sum(electrical_power([M,R],tempBin(2,:),tempBin(3,:)-input_torque,0)))
        best_torques(i) = fminbnd(energyFun, min(tempBin(3,:)),max(tempBin(3,:)));       
    end
end