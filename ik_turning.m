clear all
options.floating = true;
legs = RigidBodyManipulator('urdf/caminante_minimal.urdf',options);
nq = legs.getNumPositions(); %number of DOF

pelvis = legs.findLinkId('pelvis');
l_foot = legs.findLinkId('l_foot');
r_foot = legs.findLinkId('r_foot');
% kc1 = WorldEulerConstraint(legs,pelvis,[0;0;0],[0;0;0]);
% kc2 = WorldPositionConstraint(legs,pelvis,[0;0;0],[0.1;-.1;nan],[0.1;-.1;nan]);
% kc3 = WorldPositionConstraint(legs,l_foot,[0;0;0],[0;0.2;0],[0;0.2;0]);
% kc4 = WorldPositionConstraint(legs,r_foot,[0;0;0],[0;-0.2;0],[0;-0.2;0]);

%kc1r = WorldEulerConstraint(legs,pelvis,[-0.05;-0.05;-0.05],[0.05;0.05;0.05]); %about upright
kc2r = WorldEulerConstraint(legs,l_foot,[0;0;0],[0;0;0]); %flat feet
kc3r = WorldEulerConstraint(legs,r_foot,[0;0;-30/180*pi],[0;0;-30/180*pi]);

lean_amount = 0;
%kc1p = WorldPositionConstraint(legs,pelvis,[0;0;0],[0;lean_amount-0.01;0],[0;lean_amount+0.01;0]);
kc2p = WorldPositionConstraint(legs,l_foot,[0;0;0],[0;0.1;0.09],[0;.1;0.1]); %z must be magic number
%kc3p = WorldPositionConstraint(legs,r_foot,[0;0;0],[0;-.1;0.09],[0;-.1;0.1]);
%kc1c = WorldCoMConstraint(legs,[0;lean_amount-0.01;nan],[0;lean_amount+0.01;nan]);

q_nom = zeros(nq,1);
q_seed = q_nom+1e-2*randn(nq,1);
ikoptions = IKoptions(legs);
ikoptions = ikoptions.setDebug(true);
ikoptions = ikoptions.setMex(false);
ikoptions = ikoptions.setMajorIterationsLimit(5000);


ikproblem = InverseKinematics(legs,q_nom,kc2r,kc3r,kc2p);
ikproblem = ikproblem.setQ(ikoptions.Q);
%ikproblem = ikproblem.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_MajorIterationsLimit);

ikproblem = ikproblem.setSolverOptions('fmincon','Algorithm','sqp');
ikproblem = ikproblem.setSolverOptions('fmincon','MaxIter',500);

tic;
[qik,F,info,infeasible_cnstr_ik] = ikproblem.solve(q_seed);
toc

v = legs.constructVisualizer();
v.inspector([qik;zeros(nq,1)])

%atlas_fp.mat: 72x1 (pos+vel states)
% Try using InverseKinematics instead of inverseKin to avoid snopt

% Constraints:
%   PostureConstraint: more limited joint angles
%   WorldCoMConstraint: CoM coords