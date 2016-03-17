%% Ari Goodman
%% 3/17/2016
%example
%ik_combined([0, .1, 0.03]', [.5, -.1, 0.06]', [nan,0,nan]', [.02,.02,.02]')

function ik_combined(LPos, RPos, COMPos, Tol)
options.floating = true;
legs = RigidBodyManipulator('urdf2/Legs.urdf',options);
nq = legs.getNumPositions(); %number of DOF

pelvis = legs.findLinkId('pelvis');
l_foot = legs.findLinkId('l_foot');
r_foot = legs.findLinkId('r_foot');
l_toe = legs.findLinkId('l_foot');
r_toe = legs.findLinkId('r_foot');

kc1r = WorldEulerConstraint(legs,pelvis,-Tol,Tol); %about upright
kc2r = WorldEulerConstraint(legs,l_foot,[pi/2;0;nan]-Tol,[pi/2;0;nan]+Tol); %flat feet
kc3r = WorldEulerConstraint(legs,r_foot,[pi/2;0;nan]-Tol,[pi/2;0;nan]+Tol);
kc4r = WorldEulerConstraint(legs,l_toe,[pi/2;0;nan]-Tol,[pi/2;0;nan]+Tol);
kc5r = WorldEulerConstraint(legs,r_toe,[pi/2;0;nan]-Tol,[pi/2;0;nan]+Tol);
%0 .1 0.09, 0 .1 .1
kc1p = WorldPositionConstraint(legs,l_foot,[0;0;0],LPos-Tol,LPos+Tol);
kc2p = WorldPositionConstraint(legs,r_foot,[0;0;0],RPos-Tol,RPos+Tol);

kc1c = WorldCoMConstraint(legs,COMPos-Tol,COMPos+Tol);

q_nom = zeros(nq,1);
q_seed = q_nom+1e-2*randn(nq,1);
ikoptions = IKoptions(legs);
ikoptions = ikoptions.setDebug(true);
ikoptions = ikoptions.setMex(false);
ikoptions = ikoptions.setMajorIterationsLimit(5000);
ikproblem = InverseKinematics(legs,q_nom,kc1c,kc1r,kc2r,kc3r,kc4r,kc5r,kc1p,kc2p);
ikproblem = ikproblem.setQ(ikoptions.Q);

ikproblem = ikproblem.setSolverOptions('fmincon','Algorithm','sqp');
ikproblem = ikproblem.setSolverOptions('fmincon','MaxIter',500);

tic;
[qik,F,info,infeasible_cnstr_ik] = ikproblem.solve(q_seed);

v = legs.constructVisualizer();
v.inspector([qik;zeros(18,1)])
end