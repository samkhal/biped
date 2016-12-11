%% Ari Goodman
%% 11/5/2016
%% calculates the theoretical energy savings to a given trajectory by adding a cam to the joint to be tested.

%% Inputs:
% states: trajectory with position and speeds of joints in rad and rad/s
% torques: torque trajectory of joints in Nm
% joint_to_be_tested: string representing joint index

function energy = energy_savings_cam(states, torques, joint_to_be_tested)
gearbox = 111;
motor_values = [680; 14; .527;]; %rpm/V  %mNm/A %ohm (big motor)
if strcmp(joint_to_be_tested, 'kny')
    joints_s = [25, 30];
    joints_t = [1, 6]; 
    joints_a = [9, 14];
else
      disp('Error. Valid options for joints are kny');
      return;
end
M_motor_value_array = repmat(motor_values', 2,1); %only test 1 pair of motors at a time

dt = 1e-03; %s
%A_angle = pi - states(joints_a,:); %motor angles FOR LOGANS
A_angle = states(joints_a,:); %motor angles FOR SAMS

S_speeds = states(joints_s,:); %motor speeds
S_speeds = S_speeds .* 60/(2*pi); %rad/s -> rpm
torques = torques(joints_t,:) .* 1000; %Nm -> mNm, motor torques

energy = zeros(size(M_motor_value_array,1), 2, length(S_speeds));
spring_torque = zeros(size(M_motor_value_array,1), length(S_speeds));
power_w_spring = zeros(size(M_motor_value_array,1), length(S_speeds));
power_wo_spring = zeros(size(M_motor_value_array,1), length(S_speeds));
%for each motor, for each time step, calculate the spring torque and use it
%in conjuction with the applied motor torque to get energy.
for i = 1:size(M_motor_value_array,1)
    for j = 1:length(S_speeds)
        spring_torque(i,j) = angle2torqueCams(A_angle(i,j));
        power_w_spring(i, j) = electrical_power(M_motor_value_array(i,2), M_motor_value_array(i,3), gearbox*S_speeds(i,j), 1/gearbox*abs(torques(i,j)+spring_torque(i,j))); %power, W
        power_wo_spring(i, j) = electrical_power(M_motor_value_array(i,2), M_motor_value_array(i,3), gearbox*S_speeds(i,j), 1/gearbox*abs(torques(i,j))); %power, W
        energy(i, 1, j+1) = energy(i, 1, j) + dt*power_w_spring(i, j); %electrical energy with springs J
        energy(i, 2, j+1) = energy(i, 2, j) + dt* power_wo_spring(i, j); %electrical energy without springs J
    end
    power_stats = [mean(power_w_spring(i,:)) max(power_w_spring(i, :))]; %optional power stats
end

total_savings = (energy(1, 2, end)+energy(2, 2, end)) - (energy(1, 1, end)+energy(2, 1, end));
percent_savings=100*total_savings/(energy(1, 2, end)+energy(2, 2, end));
disp(sprintf('The spring saves %0.2f J and %0.2f%%!', total_savings, percent_savings))

%plot
energy_plot_spring = reshape(energy(1,1,:),1,length(energy(1,1,:)));
energy_plot_no_spring = reshape(energy(1,2,:),1,length(energy(1,2,:)));
plot(energy_plot_spring);
hold on;
plot(energy_plot_no_spring);
ylabel('Total Energy (J)');
xlabel('time (ms)');
title(strcat('Energy over Time for: .', joint_to_be_tested, '. for cams'));
figure;
plot((power_w_spring(1,:)+power_w_spring(2,:))/2);
hold on;
plot((power_wo_spring(1,:)+power_wo_spring(2,:))/2);
ylabel('Instantaneous Power (W)');
xlabel('time (ms)');
title(strcat('Power over Time for: .', joint_to_be_tested, '. for cams'));
end