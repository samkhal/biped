%Ari Goodman
%10/26/2016
%calculates the theoretical energy savings by adding a spring to the knee
%and ankle.

%example:

%S: matrix of joint positions (rad) and speeds (rad/s)
%T: matrix of torques, Nm
%max_spring_torque = .77; %in-lb
%spring_offset = 0.022; %radians

function energy = energy_savings(S, T, max_spring_torque, spring_offset)

joints_s = [23, 28];%only need knees 25,30 OR hipx 23 28 OR akx 27 32; hipy 24 29; 26 31 aky TODO make cleaner
joints_t = [4, 9]; %only need knees 1,6 OR hipx 4 9; akx 2 7; hipy 3 8; 5 10 aky
joints_a = [7, 12]; %only need knees 9,14 OR hipx 7 12; akx 11 16  ; hipy 8 13 ; 10 15 aky

inlb2mNm = 112.984829333;
deg2rad = 0.0174533;
k = max_spring_torque/90*(inlb2mNm)/(deg2rad); %mNm/rad
t = 1e-03; %s
m = [680; 14; .527;]; %rpm/V  %mNm/A %ohm
M = repmat(m', 2,1); %only test 1 pair of motors at a time
A = S(joints_a,:);
A(1,:) = A(1,:) - min(A(1,:)) + spring_offset; %attempt to zero springs
A(2,:) = A(2,:) - min(A(2,:)) + spring_offset;

S = S(joints_s,:);
S = S .* 60/(2*pi); %rad/s -> rpm
T = T(joints_t,:) .* 1000; %Nm -> mNm

S = S.*128; %gear box
T = T.*1/128; 

energy = zeros(size(M,1), 2, length(S));
power = zeros(size(M,1), length(S));
spring = zeros(size(M,1), length(S));
%for each motor, for each time step, calculate the spring torque and use it
%in conjuction with the applied motor torque to get energy.
for i = 1:size(M,1)
    for j = 2:length(S)
        spring(i,j) = A(i,j)*k;
        energy(i, 1, j) = energy(i, 1, j-1) + t*electrical_power(M(i,1), M(i,2), M(i,3), S(i,j), abs(T(i,j)+spring(i,j))); %electrical energy with springs J
        energy(i, 2, j) = energy(i, 2, j-1) + t*electrical_power(M(i,1), M(i,2), M(i,3), S(i,j), abs(T(i,j))); %electrical energy without springs J
        power(i, j) = electrical_power(M(i,1), M(i,2), M(i,3), S(i,j), abs(T(i,j)+spring(i,j))); %power, W
    end
    power_stats = [mean(power(i,:)) max(power(i, :))]; %optional power stats
end

%plot
energy_plot_spring = reshape(energy(1,1,:),1,length(energy(1,1,:)));
energy_plot_no_spring = reshape(energy(1,2,:),1,length(energy(1,2,:)));
plot(energy_plot_spring);
hold on;
plot(energy_plot_no_spring);
ylabel('Total Energy (J)');
xlabel('time (ms)');



end
