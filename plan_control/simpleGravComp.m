%Ari Goodman
%3/28/2017

%Given the current position and velocities, solves for the compensation
%torque for each joint.

function torque = simpleGravComp(q,v)
   options.floating = true;
   
options.ignore_self_collisions = true;
options.ignore_friction = true;
   options.dt = 0.001;
   options.terrain = RigidBodyFlatTerrain;
   options.floating = false;
    r = Caminante('urdf/caminante_minimal.urdf',options);
    r = compile(r);
    
    
    if nargin ==0
        [H C B dH dC dB] = r.manipulatorDynamics([0, 0, 0.6010, 0.0001, -0.0028, 0, -0.0143, 0.0009, 0.0009, 0.0009, -0.0091, -0.0144, 0.0009, 0.0009, 0.0009, 0.0089]', zeros(r.getNumVelocities,1));
    elseif nargin == 1
        [H C B dH dC dB] = r.manipulatorDynamics(q, zeros(r.getNumVelocities,1));
    else
        [H C B dH dC dB] = r.manipulatorDynamics(q, v);
    end
    
%H(q)*vd+C(q,v,f_ext)=B(q,v)*tau
torque = C;
end


%{
-0.0000
    0.0000
  101.9259
    0.0694
   -0.3129
   -0.0000
   -1.3288
   -0.0105
   -0.0094
   -0.0547
   -0.0000
   -1.2815
   -0.0088
   -0.0077
   -0.0541
   -0.0000
%}