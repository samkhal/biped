function runCaminanteTorques
    % Simulate the (passive) dynamics of the Caminante model 

    % Load the model with a floating base
    options.floating = true;
    options.dt = 0.001;
    options.terrain = RigidBodyFlatTerrain;
    r = Caminante('urdf/Legs.urdf',options);
    %r = r.removeCollisionGroupsExcept({'heel','toe','back','front','knee','butt'});
    r = compile(r);
    
    % create torque controller
    tc = TorqueController(); % modify this class to apply diff qorques to the legs
    tc.getOutputFrame.addTransform( ... just sets the output frame of tc equal to the input frame of r
        AffineTransform( ... 
            tc.getOutputFrame, ...
            r.getInputFrame, ...
            eye(r.getNumInputs), ... % identity matrix
            zeros(r.getNumInputs,1) ... % zero vector
        ) ...
    );

    % Initialize the viewer
    v = r.constructVisualizer;
    v.display_dt = 0.02;

    % Compute a feasible set of initial conditions for the simulation (e.g. no
    % penetration)
    x0 = Point(r.getStateFrame);
    x0 = resolveConstraints(r,x0);

    % Forward simulate dynamics with visualization, then playback at realtime
    S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
    output_select(1).system=1;
    output_select(1).output=1;
    
    sys = mimoCascade(r,v,[],[],output_select);
    sys = cascade(tc,sys); % prepend the torque controller
    
    warning(S);
    traj = simulate(sys,[0 2],x0);
    playback(v,traj,struct('slider',true));

end
