
options.floating = true;
legs = RigidBodyManipulator('urdf/Legs.urdf',options);
nq = legs.getNumPositions(); %number of DOF

pelvis = legs.findLinkId('pelvis');
l_foot = legs.findLinkId('l_foot');
r_foot = legs.findLinkId('r_foot');
kc1 = WorldEulerConstraint(legs,pelvis,[0;0;0],[0;0;0]);
kc2 = WorldPositionConstraint(legs,pelvis,[0;0;0],[0.1;-.1;nan],[0.1;-.1;nan]);
kc3 = WorldPositionConstraint(legs,l_foot,[0;0;0],[0;0.2;0],[0;0.2;0]);
kc4 = WorldPositionConstraint(legs,r_foot,[0;0;0],[0;-0.2;0],[0;-0.2;0]);

q_nom = zeros(nq,1);
q_seed = q_nom+1e-2*randn(nq,1);
ikoptions = IKoptions(legs);
ikoptions = ikoptions.setDebug(true);
ikoptions = ikoptions.setMex(false);
ikoptions = ikoptions.setMajorIterationsLimit(5000);


ikproblem = InverseKinematics(legs,q_nom,kc1,kc2,kc3,kc4);
ikproblem = ikproblem.setQ(ikoptions.Q);
%ikproblem = ikproblem.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_MajorIterationsLimit);

ikproblem = ikproblem.setSolverOptions('fmincon','Algorithm','sqp');
ikproblem = ikproblem.setSolverOptions('fmincon','MaxIter',500);

tic;
[qik,F,info,infeasible_cnstr_ik] = ikproblem.solve(q_seed);
toc

v = legs.constructVisualizer();
v.inspector([qik;zeros(18,1)])

%atlas_fp.mat: 72x1 (pos+vel states)
% Try using InverseKinematics instead of inverseKin to avoid snopt

% Constraints:
%   PostureConstraint: more limited joint angles
%   WorldCoMConstraint: CoM coords