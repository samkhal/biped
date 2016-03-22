%% Ari Goodman
%% 3/15/2016
%% based on sams code

function ik_step(lean_amount)

options.floating = true;
legs = RigidBodyManipulator('urdf/Legs.urdf',options);
nq = legs.getNumPositions(); %number of DOF

pelvis = legs.findLinkId('pelvis');
l_foot = legs.findLinkId('l_foot');
r_foot = legs.findLinkId('r_foot');

max = .1; %max and min lean amount
min= -.1;

if (lean_amount>max)
    lean_amount= max;
    disp('Warning: Lean amount too large. Truncating to max');
elseif lean_amount<min
    lean_amount=min;
    disp('Warning: Lean amount too small. Truncating to min');
end

display('Leaning');
kc1r = WorldEulerConstraint(legs,pelvis,[-0.05;-0.05;-0.05],[0.05;0.05;0.05]); %about upright
kc2r = WorldEulerConstraint(legs,l_foot,[pi/2-.05;-.05;nan],[pi/2+.05;.05;nan]); %flat feet
kc3r = WorldEulerConstraint(legs,r_foot,[pi/2-.05;-.05;nan],[pi/2+.05;.05;nan]);

kc1p = WorldPositionConstraint(legs,pelvis,[0;0;0],[0;lean_amount-0.01;0],[0;lean_amount+0.01;0]);
kc2p = WorldPositionConstraint(legs,l_foot,[0;0;0],[0;0.1;0.09],[0;.1;0.1]); %z must be magic number
kc3p = WorldPositionConstraint(legs,r_foot,[0;0;0],[0;-.1;0.09],[0;-.1;0.1]);
kc1c = WorldCoMConstraint(legs,[0;lean_amount-0.01;nan],[0;lean_amount+0.01;nan]);

q_nom = zeros(nq,1);
q_seed = q_nom+1e-2*randn(nq,1);
ikoptions = IKoptions(legs);
ikoptions = ikoptions.setDebug(true);
ikoptions = ikoptions.setMex(false);
ikoptions = ikoptions.setMajorIterationsLimit(5000);
ikproblem = InverseKinematics(legs,q_nom,kc1c,kc1r,kc2r,kc3r,kc1p,kc2p,kc3p);
ikproblem = ikproblem.setQ(ikoptions.Q);

ikproblem = ikproblem.setSolverOptions('fmincon','Algorithm','sqp');
ikproblem = ikproblem.setSolverOptions('fmincon','MaxIter',500);

tic;
[qik,F,info,infeasible_cnstr_ik] = ikproblem.solve(q_seed);
toc

v = legs.constructVisualizer();
v.inspector([qik;zeros(18,1)])

end