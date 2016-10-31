%Ari Goodman
%10/28/2016
%finds the best spring and angular offset for a joint on the robot using 90
%degree springs (for now)

function [spring_k, rad_offset] = best_spring(S, Torques, joint_to_be_tested)

range(1) = max(S(1,:)) - min(S(1,:)); %find range of angles in a trajectory
range(2) = max(S(2,:)) - min(S(2,:));

if max(range(1), range(2)) >= pi/2
    disp('Error. The range of the knee is larger than 90 degrees. Please reconsider your spring choices');
else
    maximum_offset = pi/2-max(range(1), range(2)); %maximum offset for the spring
end

inlb2mNm = 112.984829333;
deg2rad = 0.0174533;
t = 1e-03; %s
m = [680; 14; .527;]; %rpm/V  %mNm/A %ohm
M = repmat(m', 2,1);

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

Speeds = S(joints_s,:); 
Speeds = Speeds .* 60/(2*pi); %rad/s -> rpm
Torques = Torques(joints_t,:) .* 1000; %Nm -> mNm

Speeds = Speeds.*128; %gear box
Torques = Torques.*1/128; 
minimum_energy_so_far = 99999999999999999; %TODO: change to a max int
%TODO: impliment a method to do a rough search, followed by a finer search with a smaller search area, and repeat (manual for now)
for temp_k = 0:.01:1 %best spring is between these two maximum torques NOTE: IF THE ANSWER REPORTS 0, CHANGE TO -1:.01:0
    hasFoundMinForK = 0;
    disp(sprintf('percent done: %d', temp_k))
    for offset = 0:.02*maximum_offset: maximum_offset %offset is between these two values
        k = temp_k/90*(inlb2mNm)/(deg2rad); 
        A = S(joints_a,:);
        A(1,:) = A(1,:) - min(A(1,:)) + offset; %attempt to zero springs
        A(2,:) = A(2,:) - min(A(2,:)) + offset; 
        energy = zeros(size(M,1), 2);
        power = zeros(size(M,1), length(Speeds));
        spring = zeros(size(M,1), length(Speeds));
        %for each motor, for each time, add up energy with and without
        %spring
        for i = 1:size(M,1)
            for j = 1:length(Speeds)
                spring(i,j) = A(i,j)*k; %ensure sign is correct. Should be added to torque to get actual motor torque required
                energy(i, 1) = energy(i, 1) + t*electrical_power(M(i,1), M(i,2), M(i,3), Speeds(i,j), abs(Torques(i,j)+spring(i,j)));
                energy(i, 2) = energy(i, 2) + t*electrical_power(M(i,1), M(i,2), M(i,3), Speeds(i,j), abs(Torques(i,j)));
                %power(i, j) = electrical_power(M(i,1), M(i,2), M(i,3), Speeds(i,j), Torques(i,j)-spring(i,j));
            end
            %power_stats = [mean(power(i,:)) max(power(i, :))]
        end
        if minimum_energy_so_far > (energy(1,1)+energy(2,1)) %update best
            hasFoundMinForK = 1;
            minimum_energy_so_far = energy(1,1)+energy(2,1);
            spring_k = temp_k;
            rad_offset = offset;
        end
    end
    if hasFoundMinForK ~=1 % end early
        return;
    end
end
end
