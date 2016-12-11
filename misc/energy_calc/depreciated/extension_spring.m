%Ari Goodman
%11/5/2016
%finds the best extension spring for the hip joint on the robot

%% Inputs:
% states: trajectory with position and speeds of joints in rad and rad/s
% torques: torque trajectory of joints in Nm

%%outputs
% spring_k in lb/in
% init_length in m

function [spring_k, init_length] = extension_spring(states, torques)
x_dist = .1524; %m
y_dist = .0254;
r_dist = 0.0508;

lbpin2mNpm = 4448.22/(0.0254);
dt = 1e-03; %s
gearbox = 111;
motor_val = [453; 21.1; 1.24;]; %rpm/V  %mNm/A %ohm m_big
joints_s = [23, 28];
joints_t = [4, 9];
joints_a = [7, 12];
Motor_Matrix = repmat(motor_val', 2,1);

Speeds = states(joints_s,:) .* 60/(2*pi); %rad/s -> rpm
Torques = torques(joints_t,:) .* 1000; %Nm -> mNm

function cost = find_spring(x)
    temp_k = x(1)*(lbpin2mNpm); %mNm/rad;
    L_0 = x(2);
    Angles = states(joints_a,:); %attempt to zero springs
    spring = ((((y_dist - r_dist*sin(Angles(:,:))).^2 + (x_dist+r_dist*cos(Angles(:,:))).^2).^.5) -L_0).* temp_k.*r_dist.*(((x_dist^2+y_dist^2)^.5)*sin(pi-atan(y_dist/x_dist)-Angles(:,:)))./(((y_dist - r_dist*sin(Angles(:,:))).^2 + (x_dist+r_dist*cos(Angles(:,:))).^2).^.5);
    energy = zeros(size(Motor_Matrix,1),1);
    %for each motor, for each time, add up energy with and without spring 
    for i = 1:size(Motor_Matrix,1)
        for j = 1:length(Speeds)
            energy(i) = energy(i) + dt*electrical_power(Motor_Matrix(i,2), Motor_Matrix(i,3), gearbox*Speeds(i,j), 1/gearbox*abs(Torques(i,j)+spring(i,j)));
        end
    end
    cost = energy(1)+energy(2);
end

x = fmincon(@find_spring, [0,.015],[],[],[],[],[-60, 0.0127], [60, .254]);
spring_k = x(1);
init_length = x(2);
end