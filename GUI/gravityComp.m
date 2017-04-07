function torque  = grav_comp(angles_state, r)
[H C B dH dC dB] = r.manipulatorDynamics(angles_state, zeros(r.getNumVelocities,1));
torque = C;
endcd 