%% Ari Goodman
%% 11/5/2016
%% calculates the theoretical energy savings to a given trajectory by adding a spring to the joint to be tested.

%% Inputs:
% states: trajectory with position and speeds of joints in rad and rad/s
% torques: torque trajectory of joints in Nm
% max_spring_torque: the maximum spring torque in inlb
% max_spring_extension: maximum spring extension in rad
% spring_offset: spring offset initial position in rad
% joint_to_be_tested: string representing joint index

%example:
%S: matrix of joint positions (rad) and speeds (rad/s)
%T: matrix of torques, Nm
%max_spring_torque = .77; %in-lb
%spring_offset = 0.022; %radians

function energy = energy_savings(states, torques, max_spring_torque, max_spring_extension, spring_offset, joint_to_be_tested)
gearbox = 111;
motor_values = [680; 14; .527;]; %rpm/V  %mNm/A %ohm (big motor)
if strcmp(joint_to_be_tested, 'kny')
    joints_s = [25, 30];
    joints_t = [1, 6]; 
    joints_a = [9, 14];
elseif strcmp(joint_to_be_tested, 'hpx')
    joints_s = [23, 28];
    joints_t = [4, 9];
    joints_a = [7, 12];
elseif strcmp(joint_to_be_tested, 'hpy')
    joints_s = [24, 29];
    joints_t = [3, 8];
    joints_a = [8, 13];
elseif strcmp(joint_to_be_tested, 'akx')
    gearbox = 128;
    motor_values = [526; 18.1; 12.4;]; %rpm/V  %mNm/A %ohm (big motor)
    joints_s = [27, 32];
    joints_t = [2, 7]; 
    joints_a = [11, 16];
elseif strcmp(joint_to_be_tested, 'aky')
    joints_s = [26, 31];
    joints_t = [5, 10];
    joints_a = [10, 15];
else
      disp('Error. Valid options for joints are kny, hpx, hpy, akx, aky');
      return;
end
M_motor_value_array = repmat(motor_values', 2,1); %only test 1 pair of motors at a time

inlb2mNm = 112.984829333;
k_spring_constant = max_spring_torque/max_spring_extension*(inlb2mNm); %mNm/rad
dt = 1e-03; %s
angles = states(joints_a,:) + spring_offset; %motor angles
if(max(max(angles(:,:))) > max_spring_extension || min(min(angles(:,:))) < 0)
    disp ('ERROR ANGLES INCORRECT');
end

speeds = states(joints_s,:).* 60/(2*pi); %rad/s -> rpm, motor speeds
torques = torques(joints_t,:) .* 1000; %Nm -> mNm, motor torques

energy = zeros(size(M_motor_value_array,1), 2, length(speeds));
spring_torque = zeros(size(M_motor_value_array,1), length(speeds));
power_w_spring = zeros(size(M_motor_value_array,1), length(speeds));
power_wo_spring = zeros(size(M_motor_value_array,1), length(speeds));
%for each motor, for each time step, calculate the spring torque and use it
%in conjuction with the applied motor torque to get energy.
for i = 1:size(M_motor_value_array,1)
    %TODO: add init frame
    for j = 1:length(speeds)
        spring_torque(i,j) = angles(i,j)*k_spring_constant;
        power_w_spring(i, j) = electrical_power(M_motor_value_array(i,2), M_motor_value_array(i,3), gearbox*speeds(i,j), 1/gearbox*abs(torques(i,j)+spring_torque(i,j))); %power, W
        power_wo_spring(i, j) = electrical_power(M_motor_value_array(i,2), M_motor_value_array(i,3), gearbox*speeds(i,j), 1/gearbox*abs(torques(i,j))); %power, W
        energy(i, 1, j+1) = energy(i, 1, j) + dt*power_w_spring(i, j); %electrical energy with springs J
        energy(i, 2, j+1) = energy(i, 2, j) + dt*power_wo_spring(i, j); %electrical energy without springs J
    end
    power_stats = [mean(power_w_spring(i,:)) max(power_w_spring(i, :))]; %optional power stats
end

total_savings = (energy(1, 2, end)+energy(2, 2, end)) - (energy(1, 1, end)+energy(2, 1, end));
percent_savings=100*total_savings/(energy(1, 2, end)+energy(2, 2, end));
disp(sprintf('The spring saves %0.2f J and %0.2f%%!', total_savings, percent_savings))
figure;
%plot
energy_plot_spring_1 = reshape(energy(1,1,:),1,length(energy(1,1,:)));
energy_plot_no_spring_1 = reshape(energy(1,2,:),1,length(energy(1,1,:)));
energy_plot_spring_2 = reshape(energy(2,1,:),1,length(energy(1,1,:)));
energy_plot_no_spring_2 = reshape(energy(2,2,:),1,length(energy(1,1,:)));
plot(energy_plot_spring_1);
hold on;
plot(energy_plot_no_spring_1);
hold on;
plot(energy_plot_spring_2);
hold on;
plot(energy_plot_no_spring_2);
ylabel('Total Energy (J)');
xlabel('time (ms)');
title(strcat('Energy over Time for: .', joint_to_be_tested, sprintf('. with %0.2f k and %0.2f offset', max_spring_torque, spring_offset)));
legend('spring','no spring');
print(strcat('Energy_', joint_to_be_tested),'-dpng');
figure;
plot((power_w_spring(1,:)+power_w_spring(2,:))/2);
hold on;
plot((power_wo_spring(1,:)+power_wo_spring(2,:))/2);
ylabel('Instantaneous Power (W)');
xlabel('time (ms)');
title(strcat('Power over Time for: .', joint_to_be_tested, sprintf('. with %0.2f k and %0.2f offset', max_spring_torque, spring_offset)));
legend('spring','no spring');
print(strcat('Power_', joint_to_be_tested),'-dpng');
end