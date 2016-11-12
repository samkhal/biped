%% Ari Goodman
%% 11/7/2016
%% calculates the theoretical energy savings to a given trajectory by adding support to the joint specified according to the specified function

%% Inputs:
% States, states for each motor over time in rad and rad/s
% State frames, names of each state
% Torques, torques for each motor over time in Nm
% torque frames, names of each torque
% joint, the name of the joint
% foo, the function that is applied at the joint to save energy
%% Outputs:
% amount saved energy in J
% percent saved, %
% max power in W
% average power in W

function [amount_saved percent_saved max_power avg_power] = energy_savings_function(states, state_frames, torques, torque_frames, foo, constants, joint)
dt = 1/1000;
all_motor_values = [680, 14, .527; 526, 18.1, 12.4;];  %rpm/V  %mNm/A %ohm big motor; small motor
all_gear_box_values = [111; 128];
keys = {'kny', 'hpy', 'hpx', 'akx', 'aky'};
vals = [{{1,1},{1,1},{1,1},{2,2},{1,1}}];
            %name, speed index, torque index, angle index, gearbox, motor
            %type
joint_dict = containers.Map(keys,vals);
joint_data = joint_dict(joint);
joints_speed = [find(strcmp(state_frames, strcat('l_leg_',joint,'dot')),1) find(strcmp(state_frames, strcat('r_leg_',joint,'dot')),1)];
joints_torque = [find(strcmp(torque_frames, strcat('l_leg_',joint)),1) find(strcmp(torque_frames, strcat('r_leg_',joint)),1)];
joints_angle = [find(strcmp(state_frames, strcat('l_leg_',joint)),1) find(strcmp(state_frames, strcat('r_leg_',joint)),1)];
motor_values = all_motor_values(joint_data{1},:);
gearbox = all_gear_box_values(joint_data{2});

for i = 1:2 %always 2 motors
    for j = 1:length(states)
        additional_torque(i,j) = foo(states(joints_angle(i),j), constants);
        power_w_energy_saving(i, j) = electrical_power([motor_values(2), motor_values(3)], gearbox*states(joints_speed(i),j), 1/gearbox*abs(torques(joints_torque(i),j)+additional_torque(i,j)), 0); %power, W
        power_wo_energy_saving(i, j) = electrical_power([motor_values(2), motor_values(3)], gearbox*states(joints_speed(i),j), 1/gearbox*abs(torques(joints_torque(i),j)), 0); %power, W
        if j == 1
            energy(i, 1, j) = dt*power_w_energy_saving(i, j); %electrical energy with es device J
            energy(i, 2, j) = dt*power_wo_energy_saving(i, j); %electrical energy without es device J          
        else
            energy(i, 1, j) = energy(i, 1, j-1) + dt*power_w_energy_saving(i, j); %electrical energy with es device J
            energy(i, 2, j) = energy(i, 2, j-1) + dt*power_wo_energy_saving(i, j); %electrical energy without es device J
        end
    end
    power_stats(i,:) = [mean(power_w_energy_saving(i,:)) max(power_w_energy_saving(i, :))]; %optional power stats
end

amount_saved = sum(energy(:,2,end) - energy(:,1,end)); %J
percent_saved = amount_saved/sum(energy(:,2,end)); % %
max_power = max(power_stats(:, 2));% W
avg_power = mean(power_stats(:,1));% W

end