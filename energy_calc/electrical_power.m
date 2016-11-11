%% Ari Goodman
%% 11/7/2016
%% calculates electrical power given torque constant,
%% terminal resistance, speed, and torque.

%% Inputs:
% motor_constants:
    % k_M           Torque constant         mNm/A
    % R             Terminal resistance     ohm
% n                 Motor Speed             rad/s  OR rpm
% M                 Motor torque            Nm OR mNm
% isMech            boolean to switch units 0  OR  1
%% Output:
%Power in W

%% Example:
%electrical_power(14,.527, 0, 639, 1) = 1.0979e+03 W
%for the motor, the stall current is 45.5 A, and thus the voltage should be
%24.1293 V.  The nominal voltage is 24 V.

function power = electrical_power(motor_constants, n, M, isMech)
k_M = motor_constants(1);
R = motor_constants(2);
if nargin == 4
    isMech = 0;
end
if ~isMech
    n = n * 30/(pi); %rad/s -> rpm 
    M = M * 1000; %Nm -> mNm
end
I = M/k_M;
power = abs(pi/30000*n*M) + R*I*I;
end