function runCaminanteComputedTorques
    %% Load Caminante
    % Load the model with a floating base
    options.floating = true;
    options.dt = 0.01;
    options.terrain = RigidBodyFlatTerrain;

    r = Caminante('urdf/Legs.urdf',options);
    r = r.removeCollisionGroupsExcept({'heel','midfoot','toe'});
    r = compile(r);
    
    %% Controller & Feedback
    % PD gains: hip1, hip2, knee, ankle1, ankle2, toe
    Kp = diag(repmat([1, 5, 6, 8, 1, 2],1,2));
    Kd = diag(repmat([1, 1, 1, 1, .1, .1],1,2));
       
    % pd feedback term
    % tau = -Kp*theta_actual - Kd*thetadot_actual
    D = zeros(r.getNumInputs,r.getNumStates);
    index = (1:r.getNumInputs)+6;

    M = eye(18);
    
    D(:,index) = -Kp*M(7:18,7:18)+diag(C(7:18));
    D(:,index + r.getNumPositions) = M(7:18,7:18)*(-Kd)+diag(C(7:18)); %e and edot magic
    fb = LinearSystem([],[],[],[],[],D);
    fb = fb.setOutputFrame(r.getInputFrame);
    fb = fb.setInputFrame(r.getStateFrame);
   
    ff = LinearSystem([],[],[],[],[],eye(r.getNumInputs));
    ff = ff.setOutputFrame(r.getInputFrame);
    coordinates = r.getStateFrame.getCoordinateNames();
    ff = ff.setInputFrame(CoordinateFrame('q_d',length(index),'d',coordinates(index)));

    r2 = cascade(ScopeSystem(r.getInputFrame,struct('linespec','r')),r);
    sys = feedback(r2,fb);
    sys = cascade(ff,sys);
    
    %% Viewer
    % Initialize the viewer
    v = r.constructVisualizer;
    v.display_dt = 0.02;
    
    x0 = Point(r.getStateFrame);
    x0 = resolveConstraints(r,x0);
    x0(1:18) = [...
    0.0183,...
   -0.0131,...
   -0.1810,...
   -0.0197,...
   -0.0002,...
    0.0002,...
    0.0880,...
   -0.5918,...
    1.7702,...
   -1.1836,...
   -0.1074,...
    0.0052,...
    0.1477,...
   -0.5798,...
    1.7587,...
   -1.1841,...
   -0.1279,...
    0.0059]';
    
    
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
    traj = simulate(sys,[0 2*.1],x0);
    playback(v,traj,struct('slider',true));
end
