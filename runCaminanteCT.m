function runCaminanteCT
    %% Load Caminante
    % Load the model with a floating base
    options.floating = true;
    options.dt = 0.01;
    options.terrain = RigidBodyFlatTerrain;

    r = Caminante('urdf/Legs.urdf',options);
%     r = r.removeCollisionGroupsExcept({'heel','toe','back','front','knee','butt'});
    r = r.removeCollisionGroupsExcept({'heel','midfoot','toe'});
%     r = r.removeCollisionGroupsExcept({});
    r = compile(r);
    
    %% Controller & Feedback
    % PD gains: hip1, hip2, knee, ankle1, ankle2, toe
%     Kp = diag(repmat([4, 50, 50, 50, 4, 20],1,2));
%     Kd = diag(repmat([.04, .15, .16, .16, .0444, .01],1,2));

    Kp = diag([10,10,10,10,10,10,10,10,10,10,10,10]);
    Kd = diag([1,1,1,1,1,1,1,1,1,1,1,1]);

    %{ 
    works for grav = -2
    Kp = diag(repmat([1, 5, 6, 8, 1, 2],1,2));
    Kd = diag(repmat([.01, .1, .1, .1, .01, .01],1,2));
    %}
    
    % feedback term
    % tau_fb = -M*Kp*theta_actual - M*Kd*thetadot_actual + C
    fb = ComputedTorqueFB(r,-Kp,-Kd);
    
    % feedforward term
    % tau_ff = M*thetaddot_desired + M*Kp*theta_desired + M*Kd*thetadot_desired
    ff = ComputedTorqueFF(r,Kp,Kd);
    coordinates = r.getStateFrame.getCoordinateNames();
    index = (1:r.getNumInputs)+6;
    ff = ff.setInputFrame(CoordinateFrame('q_d',length(index),'d',coordinates(index)));
    
    % tau = M*(thetaddot_desired - Kp*(theta_actual - theta_desired) - Kd*(thetadot_actual - thetadot_desired))) + C
    sys = feedback(r,fb);
    sys = cascade(ff,sys);
    
    %% Planner
    
    % Compute a feasible set of initial conditions for the simulation (e.g. no
    % penetration)
    
    %x0 = zeros(r.getNumPositions*2,1); %TODO
    %x0(9:18) = [0 -.5 1.5 -1 0 0 0 .5 1.5 -1]'; %FIX ME
    x0 = Point(r.getStateFrame);
    x0 = resolveConstraints(r,x0);
    %x0(3) = 0.0001;
    %x0([12,18]) = .3;
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
    
    pl = TrajectoryPlanner(sys,x0);
    sys = cascade(pl,sys);
    
    %% Viewer
    % Initialize the viewer
    v = r.constructVisualizer;
    v = r.constructVisualizer;
    v.display_dt = 0.02;    
    
    % Forward simulate dynamics with visualization, then playback at realtime
    S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
    output_select(1).system=1;
    output_select(1).output=1;
    
    sys = mimoCascade(sys,v,[],[],output_select);
    
    %% Simulate 
    warning(S);
    traj = simulate(sys,[0 2*5],x0);
    playback(v,traj,struct('slider',true));
end
