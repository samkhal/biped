%%Ari Goodman
% 11/5/2016
% finds the best torsional springs for all joints (does not directly tell
% you when infeasible)

%% inputs
%  states:              rad, rad/s for each joint
%  torques:             Nm for each joint
%  spring ranges:        rad, max extension fo torsion spring

%% output
% spring k          max force
% rad offset        offet in radians

function [spring_k, rad_offset] = all_springs(states, torques, spring_ranges)

    valueSet =   {'kny', 'hpx', 'hpy', 'akx', 'aky'};
    keySet = [1,2,3,4,5];
    motorMap = containers.Map(keySet,valueSet);

    spring_k = zeros(length(keySet),1);
    rad_offset = zeros(length(keySet),1);

    for motor = 1:length(keySet)
        [spring_k(motor), rad_offset(motor)] = best_spring(states, torques, spring_ranges(motor), motorMap(motor));
    end
    for motor = 1:length(keySet)
        energy_savings(states, torques, spring_k(motor), spring_ranges(motor), rad_offset(motor), motorMap(motor));
    end
end