%Ari Goodman
%10/26/2016
%calculates electrical energy (J) for each motor given speed (rad/s) and torque (Nm) of each
%motor and the motor type and time step
%S is in rad/s
%T is in Nm
%M [ k_n rpm/V, k_M mNm/A, R Ohm]

function energy = total_energy(S, T)
t = 1e-03; %s
m = [680; 14; .527;]; %rpm/V  %mNm/A %ohm
M = repmat(m', 10,1); %each motor is the same
S = S(23:32,:); %only need these rows
T(:,:) = T([4,3,1,5,2,9,8,6,10,7],:); %different order on torques
S = S .* 60/(2*pi); %rad/s -> rpm
T = T .* 1000; %Nm -> mNm

S = S .* 128; %gear box
T = T .* 1/128; 

energy = zeros(size(M,1), 1);
power = zeros(size(M,1), length(S));
%for each motor, for each time, add up the energy
for i = 1:size(M,1)
    for j = 1:length(S)
        energy(i) = energy(i) + t*electrical_power(M(i,1), M(i,2), M(i,3), abs(S(i,j)), abs(T(i,j)));
        power(i, j) = electrical_power(M(i,1), M(i,2), M(i,3), abs(S(i,j)), abs(T(i,j))); %optional
    end
    power_stats = [i, mean(power(i,:)) max(power(i, :))] %optional
end
end