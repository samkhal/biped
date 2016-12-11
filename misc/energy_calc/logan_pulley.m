% Sam Khalandovsky
% Generates and saves a pulley for Logan's gravity comp prototype

k = 3; %lbf/in
x = 8; %in
y = 0;

tau = @(theta) 2.2*cos(theta); %lbf*in
max_force = 7; %lbf

% tau should be a decreasing function over the range of interest.
% The maximum torque <tau(0)> is achieved at theta=0 with max_force spring
% force
pulley_ps = generate_pulley(tau, x, y, pi/2, 0.001, max_force, k, true); 
csvwrite('loganpulley.csv',pulley_ps)