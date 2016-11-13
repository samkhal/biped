load five_step_walk_v3.mat

% get all positons at specific index
traj.pos(500);
traj.vel(500);
traj.torque(500);

% Get joint names
fieldnames(traj.torque);

% Get specific value
traj.vel(500).base_x;

var = 'base_x';
traj.vel(500).(var);

% Get all velocities of a joint as an vector
x_vels = [traj.vel.base_x];

% Get all velocities of all joints as a matrix
vel_states = cell2mat(squeeze(struct2cell(traj.vel)));



