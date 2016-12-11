% Sam Khalandovsky
% Generates and a knee pulley, then calculates it's function

%% Setup parameters
anchor_dist = si(7.54,'in'); %m
anchor_offset = si(0.47,'in');

%torque_vals = [0.5, 0.5, 11.2, 11.2];%N*m
torque_vals = [0.0, 0.0, 11.2, 11.2];%N*m
torque_bps = [0, 1.223, 1.427, 2];%rad
theta_max = pi/2;
tau = @(theta)  interp1(torque_bps, torque_vals, theta_max-theta, 'linear'); %N*m
tau = @(theta)  2*(pi/2-theta);
min_radius = 0.02;% 1 in =0.0254m; %m
spring_max = max(torque_vals)/min_radius; %N
spring_k = spring_max/min_radius*1; %approx

%% Generate pulley

% tau should be a decreasing function over the range of interest.
% The maximum torque <tau(0)> is achieved at theta=0 with max_force spring
% force
pulley_ps = generate_pulley(tau, anchor_dist, anchor_offset, theta_max, pi/2/100, spring_max, spring_k, true); 
pulley_ps = pulley_ps(2:end,:);

%% Calculate pulley function
% create args for get_pulley_torque_fun
alpha_diffs = diff(pulley_ps);
alpha_dists = hypot(alpha_diffs(:,1),alpha_diffs(:,2));
px_1 = pulley_ps(1,1) - (pulley_ps(1,1)+ anchor_offset)/(pulley_ps(1,2) + anchor_dist) * pulley_ps(1,2);

fun = get_pulley_torque_fun(anchor_dist, anchor_offset, theta_max, spring_max, spring_k, px_1, alpha_dists, 0);


