%Ari Goodman
%3/28/2017

%Given the current position and velocities, solves for the compensation
%torque for each joint.

function torque = simpleGravComp(q,v)
%    options.floating = true;
%    options.dt = 0.001;
%    options.terrain = RigidBodyFlatTerrain;
%     r = Caminante('urdf/caminante_minimal.urdf',options);
%     r = compile(r);
    global r torques_temp states_temp
    if nargin ==0
        [H C B dH dC dB] = r.manipulatorDynamics([0, 0, 0.6010, 0.0001, -0.0028, 0, -0.0143, 0.0009, 0.0009, 0.0009, -0.0091, -0.0144, 0.0009, 0.0009, 0.0009, 0.0089]', zeros(r.getNumVelocities,1));
    elseif nargin == 1
        [H C B dH dC dB] = r.manipulatorDynamics(q, zeros(r.getNumVelocities,1));
    else
        [H C B dH dC dB] = r.manipulatorDynamics(q, v);
    end
    
%H(q)*vd+C(q,v,f_ext)=B(q,v)*tau
torque = C(7:16);
torques_temp = [torques_temp;C'];
states_temp = [states_temp;q'];
end