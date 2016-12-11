% Sam Khalandovsky
% Get initial pulley values from knee_pulley, optimize them, and draw the
% output pulley

knee_pulley

[theta, rad] = cart2pol(pulley_ps(:,1),pulley_ps(:,2));
theta = 3*pi/2-theta;
d_theta = theta_max/100;

[p_vals, frac] = optimize_pulley_radius(traj, [anchor_dist, anchor_offset, theta_max, spring_max, spring_k, theta(1), d_theta, rad']);
t_fun = get_pulley_torque_fun_radius(p_vals(1),p_vals(2),p_vals(3),p_vals(4),p_vals(5),p_vals(6),p_vals(7),p_vals(8:end)',1);