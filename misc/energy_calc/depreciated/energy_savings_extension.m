%% Ari Goodman
%% 11/5/2016
%% calculates the theoretical energy savings to a given trajectory by adding an extension spring to the joint to be tested.

%% Inputs:
% states: trajectory with position and speeds of joints in rad and rad/s
% torques: torque trajectory of joints in Nm
% L_0:      initial spring extension in m
% spring constant:  lbf/in      OR mN/m
% isInnmn:      boolean         determines whether to convert or not

function energy = energy_savings_extension(states, torques, L_0, spring_constant, isntInmNperm)
x_dist = .1524;
y_dist = .0254;
r_dist = 0.0508;
gearbox = 111;
motor_values = [680; 14; .527;]; %rpm/V  %mNm/A %ohm (big motor)
joints_s = [23, 28];
joints_t = [4, 9];
joints_a = [7, 12];
M_motor_value_array = repmat(motor_values', 2,1); %only test 1 pair of motors at a time
lbpin2mNpm = 4448.22/(0.0254);
if isntInmNperm
    k_spring_constant = spring_constant*lbpin2mNpm; %mN/m
else
    k_spring_constant = spring_constant;
end
dt = 1e-03; %s
angles = states(joints_a,:); %motor angles

speeds = states(joints_s,:).* 60/(2*pi); %rad/s -> rpm, motor speeds
torques = torques(joints_t,:) .* 1000; %Nm -> mNm, motor torques

energy = zeros(size(M_motor_value_array,1), 2, length(speeds));
power_w_spring = zeros(size(M_motor_value_array,1), length(speeds));
power_wo_spring = zeros(size(M_motor_value_array,1), length(speeds));
%for each motor, for each time step, calculate the spring torque and use it
%in conjuction with the applied motor torque to get energy.
spring_torque = ((((y_dist - r_dist*sin(angles(:,:))).^2 + (x_dist+r_dist*cos(angles(:,:))).^2).^.5) -L_0).* k_spring_constant.*r_dist.*(((x_dist^2+y_dist^2)^.5)*sin(pi-atan(y_dist/x_dist)-angles(:,:)))./(((y_dist - r_dist*sin(angles(:,:))).^2 + (x_dist+r_dist*cos(angles(:,:))).^2).^.5);
for i = 1:size(M_motor_value_array,1)
    for j = 1:length(speeds)
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
title(strcat('extension Energy over Time for: hpx', sprintf(' with %0.2f k and %0.2f L0', spring_constant, L_0)));
legend('spring','no spring');
print(strcat('Energy_', 'hpx extension'),'-dpng');
figure;
plot((power_w_spring(1,:)+power_w_spring(2,:))/2);
hold on;
plot((power_wo_spring(1,:)+power_wo_spring(2,:))/2);
ylabel('Instantaneous Power (W)');
xlabel('time (ms)');
title(strcat('extension Power over Time for: hpx', sprintf('. with %0.2f k and %0.2f offset', spring_constant, L_0)));
legend('spring','no spring');
print(strcat('Power_', 'hpx extension'),'-dpng');
end