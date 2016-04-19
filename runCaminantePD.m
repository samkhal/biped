function runCaminantePD
    %% Load Caminante
    % Load the model with a floating base
    options.floating = true;
    options.dt = 0.001;
    options.terrain = RigidBodyFlatTerrain;

    r = Caminante('urdf/Legs.urdf',options);
    %r = r.removeCollisionGroupsExcept({'heel','toe','back','front','knee','butt'});
    r = r.removeCollisionGroupsExcept({'heel','midfoot','toe'});
    r = compile(r);
    
    %% Controller & Feedback
    % PD gains: hip1, hip2, knee, ankle1, ankle2, toe
    Kp = diag(repmat([20, 20, 40, 40, 15, .6],1,2));
    Kd = diag(repmat([1,  2,  2,  2,  1,  .3],1,2));
    
    % pd feedback term
    % tau = -Kp*theta_actual - Kd*thetadot_actual
    D = zeros(r.getNumInputs,r.getNumStates);
    index = (1:r.getNumInputs)+6;
    D(:,index) = -Kp;
    D(:,index + r.getNumPositions) = -Kd;
    fb = LinearSystem([],[],[],[],[],D);
    fb = fb.setOutputFrame(r.getInputFrame);
    fb = fb.setInputFrame(r.getStateFrame);
    
    % pd feedforward term
    % tau = Kp*theta_desired
    ff = LinearSystem([],[],[],[],[],Kp*eye(r.getNumInputs));
    ff = ff.setOutputFrame(r.getInputFrame);
    coordinates = r.getStateFrame.getCoordinateNames();
    ff = ff.setInputFrame(CoordinateFrame('q_d',length(index),'d',coordinates(index)));
    
    % tau = Kp*(theta_desired - theta_actual) - Kd*thetadot_actual
    
    r2 = cascade(ScopeSystem(r.getInputFrame,struct('linespec','r')),r);
    sys = feedback(r2,fb);
    sys = cascade(ff,sys);
    
    
    %% Viewer
    % Initialize the viewer
    v = r.constructVisualizer;
    v.display_dt = 0.02;

    % Compute a feasible set of initial conditions for the simulation (e.g. no
    % penetration)
    x0 = Point(r.getStateFrame);
    x0 = resolveConstraints(r,x0);
    x0(3) = 0.05;
    x0([12,18]) = .3;

    % Forward simulate dynamics with visualization, then playback at realtime
    S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
    output_select(1).system=1;
    output_select(1).output=1;
    
    sys = mimoCascade(sys,v,[],[],output_select);
    
    %% Planner
    pl = TrajectoryPlanner(sys,x0);
    sys = cascade(pl,sys);
    
    %% Simulate 
    warning(S);
    traj = simulate(sys,[0 8.75],x0);
    playback(v,traj,struct('slider',true));
end
