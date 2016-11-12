%Ari Goodman
%11/7/2016
%finds the best coefficients for a function for a joint on the robot

%% Inputs:
% States, states for each motor over time in rad and rad/s
% State frames, names of each state
% Torques, torques for each motor over time in Nm
% torque frames, names of each torque
% joint, the name of the joint
% foo, the function that is applied at the joint to save energy
% constant range, range of constants for foo

%%outputs
% coefficients to be used in in foo


function output_constants = find_optimal_constants(states, torques, state_frames, torque_frames, foo, constant_range,joint)

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

function cost = cost_function(x)
    energy = zeros(2,1);
    additional_torque(:,:) = foo(states(joints_angle(:),:), x);
    %for each motor, for each time, add up energy with and without spring 
    for i = 1:2
        for j = 1:length(states(joints_speed(i),:))
            energy(i) = energy(i) + dt*electrical_power([motor_values(2), motor_values(3)], gearbox*states(joints_speed(i),j), 1/gearbox*abs(torques(joints_torque(i),j)+additional_torque(i,j)), 0);
        end
    end
    cost = energy(1)+energy(2);
    if(min(min(additional_torque(:,:)))<=0.01) %2*k*d*r, 200 N/m * 0.002 m * 0.0127
        cost = cost+ abs(1000*((min(min(additional_torque(:,:))))-.01));
    end
end

x = fmincon(@cost_function, zeros(1,length(constant_range)),[],[],[],[],constant_range(1,:), constant_range(2,:));
output_constants = x(1:length(constant_range));
end