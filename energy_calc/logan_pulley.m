% logan cam

k = 3; %lbf/in
x = 8;
y = 0;

tau = @(theta) 2.2*cos(theta);
max_force = 7;

% tau should be a decreasing function over the range of interest
pulley_ps = generate_pulley(tau, x, y, pi/2, 0.001, max_force, k, true); 
csvwrite('logancam.csv',pulley_ps)