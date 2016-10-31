%Ari Goodman
%10/26/2016
%calculates electrical power given speed constant, torque constant,
%terminal resistance, speed, and torque. Be sure to check units.

%Example:
%electrical_power(680,14,.527, 0, 639) = 1.0979e+03 W
%for the motor, the stall current is 45.5 A, and thus the voltage should be
%24.1293 V.  The nominal voltage is 24 V.

function power = electrical_power(k_n, k_M, R, n, M)
% k_n = 680; %rpm/V
% k_M = 14; %mNm/A
% R = .527; %ohm
% n is in rpm
% M is in mNm
I = M/k_M;
power = abs(pi/30000*n*M) + R*I*I;
end
