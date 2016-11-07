%% Ari Goodman
%% 11/5/2016
%% calculates electrical power given torque constant,
%% terminal resistance, speed, and torque.

%% Inputs:
% Torque constant k_M = 14; %mNm/A
% Terminal resistance R = .527; %ohm
% Motor speed n is in rpm
% Motor torque M is in mNm
%% Output:
%Power in W

%% Example:
%electrical_power(14,.527, 0, 639) = 1.0979e+03 W
%for the motor, the stall current is 45.5 A, and thus the voltage should be
%24.1293 V.  The nominal voltage is 24 V.

function power = electrical_power(k_M, R, n, M)
I = M/k_M;
power = abs(pi/30000*n*M) + R*I*I;
end