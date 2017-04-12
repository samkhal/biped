% Export states and torques from the variables produced by
% runCaminanteWalking into a mat file for use outside Drake

function exportTrajectory(filename,notes)
ytraj = evalin('base','ytraj');
r = evalin('base','r');


states = ytraj.xx;
times = ytraj.tt;
traj.times = times;

state_frames = r.getOutputFrame.getCoordinateNames;
torque_frames = r.getInputFrame.getCoordinateNames;

assert(size(states,2)==size(times,2));
global torques
if any(isnan(torques))
    assert(size(states,2)<find(isnan(torques(1,:)),1)); % nans in torque should start after the length of states
end
torques = torques(:,1:size(states,2));

nq = length(state_frames)/2;
for i = 1:nq
    pos.(state_frames{i}) = states(i,:)';
    vel.(state_frames{i+nq}(1:end-3)) = states(i+nq,:)'; %chop off the "dot" at end of velocity names
end
traj.pos = structofarrays2arrayofstructs(pos);
traj.vel = structofarrays2arrayofstructs(vel);

for i = 1:length(torque_frames)
    torque.(torque_frames{i}) = torques(i,:)';
end
traj.torque = structofarrays2arrayofstructs(torque);

% notes variable should have a string with relevant info
traj.notes = notes;

traj.walking_options = evalin('base','walking_options');
traj.robot_options = evalin('base','robot_options');
traj.com = evalin('base','com');
traj.footstep_plan = evalin('base','footstep_plan');
traj.walking_plan_data = evalin('base','walking_plan_data');
traj.ytraj = ytraj;

save(['~/biped/plan_control/traj/' filename], 'traj');
