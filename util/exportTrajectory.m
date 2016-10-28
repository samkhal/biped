% Export states and torques from the variables produced by
% runCaminanteWalking into a mat file for use outside Drake

states = ytraj.xx;
times = ytraj.tt;
assert(size(states,2)==size(times,2));
assert(size(states,2)<find(isnan(torques(1,:)),1)); % nans in torque should start after the length of states
torques = torques(:,1:size(states,2));

state_frames = r.getOutputFrame.getCoordinateNames;
torque_frames = r.getInputFrame.getCoordinateNames;

save(filename, 'states', 'times', 'torques', 'state_frames', 'torque_frames');