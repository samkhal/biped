%% Ari Goodman
%% 11/7/2016
%calculates electrical energy (J) for each motor given speed (rad/s) and torque (Nm) of each motor

%% Inputs:
% States, states for each motor over time in rad and rad/s
% State frames, names of each state
% Torques, torques for each motor over time in Nm
% torque frames, names of each torque
%% Outputs:
% energy in J
% max power in W
% average power in W
function [energy max_power average_power]= total_energy(states, state_frames, torques, torque_frames)
dt = 1/1000;
all_motor_values = [680, 14, .527; 526, 18.1, 12.4;];  %rpm/V  %mNm/A %ohm big motor; small motor
all_gear_box_values = [111; 128];
keys = {'kny', 'hpy', 'hpx', 'akx', 'aky'};
vals = [{{1,1},{1,1},{1,1},{2,2},{1,1}}];
            %name, speed index, torque index, angle index, gearbox, motor
            %type
joint_dict = containers.Map(keys,vals);

energy = zeros(length(keys)*2,1);
power = zeros(length(keys)*2, length(states));
max_power = zeros(length(keys)*2, 1);
average_power = zeros(length(keys)*2, 1);
%for each motor, for each time, add up the energy
for i = 1:length(keys)
    joint = keys{i};
    joint_data = joint_dict(joint);
    joints_speed = [find(strcmp(state_frames, strcat('l_leg_',joint,'dot')),1) find(strcmp(state_frames, strcat('r_leg_',joint,'dot')),1)];
    joints_torque = [find(strcmp(torque_frames, strcat('l_leg_',joint)),1) find(strcmp(torque_frames, strcat('r_leg_',joint)),1)];
    motor_values = all_motor_values(joint_data{1},:);
    gearbox = all_gear_box_values(joint_data{2});
    for k = 1:2
        for j = 1:length(states)
            power(k+2*(i-1), j) = electrical_power([motor_values(2), motor_values(3)], gearbox*abs(states(joints_speed(k),j)), 1/gearbox*abs(torques(joints_torque(k),j)),0);
            energy(k+2*(i-1)) = energy(k+2*(i-1)) + dt*power(k+2*(i-1),j);
        end
        max_power(k+2*(i-1)) = max(power(k+2*(i-1), :));
        average_power(k+2*(i-1)) = mean(power(k+2*(i-1), :));
    end
end
end