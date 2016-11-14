%% Ari Goodman
%% 11/13/2016
%% calculates the theoretical energy savings to a given trajectory by adding support to the joint specified according to the specified function

%% Inputs:

% trajectory

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

function [amount_saved percent_saved max_power avg_power] = energy_savings_function(trajectory, foo, constants, joint)
dt = 1/1000;
all_motor_values = [680, 14, .527; 526, 18.1, 12.4;];  %rpm/V  %mNm/A %ohm big motor; small motor
all_gear_box_values = [111; 128];
keys = fieldnames(trajectory.torque);%field names to access gearbox and motor values. hard coded and should be changed.
vals = {{1,1},{2,2},{1,1},{1,1},{1,1},{1,1},{2,2},{1,1},{1,1},{1,1}};

joint_dict = containers.Map(keys,vals);
joint_data = joint_dict(joint);

joints_velocity = [trajectory.vel.(joint)];
joints_torque = [trajectory.torque.(joint)];
joints_angle = [trajectory.pos.(joint)];
motor_values = all_motor_values(joint_data{1},:);
gearbox = all_gear_box_values(joint_data{2});

additional_torque = zeros(size(trajectory.torque,2),1);
power_w_energy_saving = zeros(size(trajectory.torque,2),1);
power_wo_energy_saving = zeros(size(trajectory.torque,2),1);
power_stats = zeros(2,1);
additional_torque(:) = foo(joints_angle, constants);
power_w_energy_saving(:) = electrical_power([motor_values(2), motor_values(3)], gearbox*joints_velocity, 1/gearbox*abs(joints_torque+additional_torque(:)'), 0); %power, W
power_wo_energy_saving(:) = electrical_power([motor_values(2), motor_values(3)], gearbox*joints_velocity, 1/gearbox*abs(joints_torque), 0); %power, W   
power_stats(:) = [mean(power_w_energy_saving(:)) max(power_w_energy_saving(:))]; %optional power stats

amount_saved = -sum(sum(power_w_energy_saving(:,:)))*dt + sum(sum(power_wo_energy_saving(:,:)))*dt; %J
percent_saved = amount_saved/(sum(sum(power_wo_energy_saving(:,:))*dt));
max_power = max(power_stats(2));% W
avg_power = mean(power_stats(1));% W

end