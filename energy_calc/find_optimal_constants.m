%Ari Goodman
%11/13/2016
%finds the best coefficients for a function for a joint on the robot

%% Inputs:

% trajectory

% States, states for each motor over time in rad and rad/s
% State frames, names of each state
% Torques, torques for each motor over time in Nm
% torque frames, names of each torque

% joint, the name of the joint
% foo, the function that is applied at the joint to save energy
% constant range, range of constants for foo

%%outputs
% coefficients to be used in in foo


function output_constants = find_optimal_constants(trajectory, foo, constant_range,joint)

dt = 1/1000; %time step
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

function cost = cost_function(x)
    power = zeros(1,size(trajectory.torque,2));
    additional_torque(:,:) = foo(joints_angle, x);
    %for each motor, for each time, add up energy with and without spring 
    tic
    power(:) = electrical_power([motor_values(2), motor_values(3)],gearbox.*joints_velocity, 1/gearbox.*abs(joints_torque+additional_torque(:)'), 0);
    cost = sum(sum(power))*dt;
    if(min(min(additional_torque(:,:)))<=0.01) %2*k*d*r, 200 N/m * 0.002 m * 0.0127
        cost = cost+ abs(1000*((min(min(additional_torque(:,:))))-.01));
    end
end

x = fmincon(@cost_function, zeros(1,size(constant_range,2)),[],[],[],[],constant_range(1,:), constant_range(2,:));
output_constants = x(1:size(constant_range,2));
end