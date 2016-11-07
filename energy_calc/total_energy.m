%% Ari Goodman
%% 11/1/2016
%calculates electrical energy (J) for each motor given speed (rad/s) and torque (Nm) of each motor

%% Inputs:
% S, states for each motor over time in rad and rad/s
% T, torques for each motor over time in Nm
%% Outputs:
% energy in J

function energy = total_energy(states, torques)
dt = 1e-03; %s
m_big = [453; 21.1; 1.24;]; %rpm/V  %mNm/A %ohm
m_small = [526; 18.1; 12.4;]; %rpm/V  %mNm/A %ohm
motor_vals = repmat(m_big', 10,1); %each motor is the same
motor_vals(5,:) = m_small; %except the small motor aky
motor_vals(10,:) = m_small; %except the small motor aky
states = states(23:32,:); %only need these rows
torques(:,:) = torques([4,3,1,5,2,9,8,6,10,7],:); %different order on torques
states = states .* 60/(2*pi); %rad/s -> rpm
torques = torques .* 1000; %Nm -> mNm

states([1,2,3,4,6,7,8,9],:) = states([1,2,3,4,6,7,8,9],:)*111; %gearbox
torques([1,2,3,4,6,7,8,9],:) = torques([1,2,3,4,6,7,8,9],:)*1/111;
states([5,10],:) = states([5,10],:)*128;
torques([5,10],:) = torques([5,10],:)*1/128;

energy = zeros(size(motor_vals,1), 1);
power = zeros(size(motor_vals,1), length(states));
%for each motor, for each time, add up the energy
for i = 1:size(motor_vals,1)
    for j = 1:length(states)
        energy(i) = energy(i) + dt*electrical_power(motor_vals(i,2), motor_vals(i,3), abs(states(i,j)), abs(torques(i,j)));
        power(i, j) = electrical_power(motor_vals(i,2), motor_vals(i,3), abs(states(i,j)), abs(torques(i,j))); %optional
    end
    power_stats = [i, mean(power(i,:)) max(power(i, :))] %optional
end
end