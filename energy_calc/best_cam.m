%Ari Goodman
%11/5/2016
%finds the best cam equation in the form A+B*x+C*x^2 for a joint on the robot

%% Inputs:
% states: trajectory with position and speeds of joints in rad and rad/s
% torques: torque trajectory of joints in Nm

%%outputs
% A,B,C coefficients to be used in angle2torquecams


function [A, B, C] = best_cam(states, torques)

dt = 1e-03; %s
gearbox = 111;
motor_val = [453; 21.1; 1.24;]; %rpm/V  %mNm/A %ohm m_big
joints_s = [25, 30];
joints_t = [1, 6]; 
joints_a = [9, 14];
Motor_Matrix = repmat(motor_val', 2,1);

Speeds = states(joints_s,:) .* 60/(2*pi); %rad/s -> rpm
Torques = torques(joints_t,:) .* 1000; %Nm -> mNm

function cost = find_cam(x)
    Angles = states(joints_a,:); %attempt to zero springs
    spring = abs((Angles(:,:).^2)).*x(3)+(Angles(:,:)).*x(2)+x(1);
    energy = zeros(size(Motor_Matrix,1),1);
    %for each motor, for each time, add up energy with and without spring 
    for i = 1:size(Motor_Matrix,1)
        for j = 1:length(Speeds)
            energy(i) = energy(i) + dt*electrical_power(Motor_Matrix(i,2), Motor_Matrix(i,3), gearbox*Speeds(i,j), 1/gearbox*abs(Torques(i,j)+spring(i,j)));
        end
    end
    cost = energy(1)+energy(2);
end

x = fmincon(@find_cam, [0,0,0],[],[],[],[],[-600000,-600000, -600000], [600000, 600000, 600000]);
A = x(1);
B = x(2);
C = x(3);
end