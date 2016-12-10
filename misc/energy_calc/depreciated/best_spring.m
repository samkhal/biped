%Ari Goodman
%11/5/2016
%finds the best torsional spring constant and angular offset for a joint on the robot

%% inputs
%  states:              rad, rad/s for each joint
%  torques:             Nm for each joint
%  spring range:        rad, max extension fo torsion spring (i.e. the
%                           angle of the uncompressed spring)
%  joint to be tested:  string, kny, aky, akx, hpx, hpy

%% output
% spring_k              best spring force in lbf NOT SPRING CONSTANT
% rad_offset            offset in radians


function [spring_k, rad_offset] = best_spring(states, torques, spring_range, joint_to_be_tested)

inlb2mNm = 112.984829333;%inlb -> mNm
dt = 1e-03; %s
gearbox = 111;
motor_val = [453; 21.1; 1.24;]; %rpm/V  %mNm/A %ohm m_big
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
    motor_val = [526; 18.1; 12.4;];
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

maximum_offset = spring_range-max(max((states(joints_a,:)))); %maximum offset for the spring
minimum_offset = -min(min((states(joints_a,:)))); %maximum offset for the spring

Motor_Matrix = repmat(motor_val', 2,1);

Speeds = states(joints_s,:) .* 60/(2*pi); %rad/s -> rpm
Torques = torques(joints_t,:) .* 1000; %Nm -> mNm

function cost = find_spring(x)
    temp_k = x(1);
    offset = x(2);
    k = temp_k/spring_range*(inlb2mNm);
    Angles = states(joints_a,:) + offset; %attempt to zero springs
    spring = Angles(:,:).*k;
    energy = zeros(size(Motor_Matrix,1),1);
    %for each motor, for each time, add up energy with and without spring 
    for i = 1:size(Motor_Matrix,1)
        for j = 1:length(Speeds)
            energy(i) = energy(i) + dt*electrical_power(Motor_Matrix(i,2), Motor_Matrix(i,3), gearbox*Speeds(i,j), 1/gearbox*abs(Torques(i,j)+spring(i,j)));
        end
    end
    cost = energy(1)+energy(2);
end

x = fmincon(@find_spring, [0,(minimum_offset+maximum_offset)/2],[],[],[],[],[-600, minimum_offset], [600, maximum_offset]);
spring_k = x(1);
rad_offset = x(2);
end