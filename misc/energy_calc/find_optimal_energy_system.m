%%Ari Goodman
% 11/13/2016
% finds the best energy saving system of the type specified for the joints
% specified

%% inputs
% trajectory

% States, states for each motor over time in rad and rad/s
% State frames, names of each state
% Torques, torques for each motor over time in Nm
% torque frames, names of each torque

% joint, the name of the joint
% foo, the function that is applied at the joint to save energy
% constant range, range of constants for foo

%outputs:
% all constants: all the constants in the provided equation
% all savings:  all the savigns such as total, percent, max power, and mean
% power

function [all_constants all_savings] = find_optimal_energy_system(trajectory, foo, constant_range, joints)

    %find the optimal constants in the equation
    all_constants = zeros(length(joints),length(constant_range));
    for motor = 1:length(joints)
        all_constants(motor, :) = find_optimal_constants(trajectory, foo, constant_range, joints{motor});
    end
    %find the savings for the constants and equation
    for motor = 1:length(joints)
        [amount(motor) perc(motor) max_pow(motor) mean_pow(motor)] = energy_savings_function(trajectory, foo, all_constants(motor,:), joints{motor})
    end
    all_savings = [amount(:) perc(:) max_pow(:) mean_pow(:)];
end