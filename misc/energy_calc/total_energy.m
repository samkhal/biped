%% Ari Goodman
%% 11/13/2016
%calculates electrical energy (J) for each motor given speed (rad/s) and
%torque (Nm) of each motor in the form of a trajectory. Currently, motor
%values and gearbox values are hard coded.

%% Inputs:
% trajectory Trajectory struct with times, pos, vel, torque, notes, etc.
%% Outputs:
% energy in J
% max power in W
% average power in W
function [energy, max_power, average_power]= total_energy(trajectory)
dt = trajectory.times(2) - trajectory.times(1); %time step
all_motor_values = [453, 21.1, 1.24; 352, 27.2, 28.7;];  %rpm/V  %mNm/A %ohm big motor; small motor
all_gear_box_values = [111; 128];
keys = fieldnames(trajectory.torque); %field names to access gearbox and motor values. hard coded and should be changed.
vals = {{1,1},{2,2},{1,1},{1,1},{1,1},{1,1},{2,2},{1,1},{1,1},{1,1}};
joint_dict = containers.Map(keys,vals);

energy = zeros(size(keys,1),1);
power = zeros(size(keys,1), size(trajectory.torque,2));
max_power = zeros(size(keys,1), 1);
average_power = zeros(size(keys,1), 1);
%for each motor,add up the power. multiply by dt to get energy
for i = 1:size(keys,1)
    joint = keys{i};
    joint_data = joint_dict(joint);
    motor_values = all_motor_values(joint_data{1},:);
    gearbox = all_gear_box_values(joint_data{2});
    power(i,:) = electrical_power([motor_values(2), motor_values(3)],[trajectory.vel.(joint)].*gearbox,[trajectory.torque.(joint)].*(1/gearbox), 0);
    energy(i) = sum(power(i,:))*dt;
    max_power(i) = max(power(i, :));
    average_power(i) = mean(power(i, :));
end
end