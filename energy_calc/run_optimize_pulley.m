% Sam Khalandovsky
% Get initial pulley values from knee_pulley, optimize them, and draw the
% output pulley

knee_pulley

[p_vals, frac] = optimize_pulley(traj, [anchor_dist, anchor_offset, theta_max, spring_max, spring_k, px_1, alpha_dists']);
t_fun = get_pulley_torque_fun(p_vals(1),p_vals(2),p_vals(3),p_vals(4),p_vals(5),p_vals(6),p_vals(7:end),1);