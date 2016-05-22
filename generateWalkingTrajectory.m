function outputTraj = generateWalkingTrajectory(LPos, RPos, COMPos, Tol)
%% Ari Goodman
%% 4/11/2016
%% Generates Walking Trajectory for runCaminantePD 

if nargin<1
      LPos = [0, .1, 0.05;...
          0, .1, 0.05;...
          0, .1, 0.05;...
          0, .1, 0.05;...
          0, .1, 0.05;...%EDITED
          0, .1, 0.05;...
          0, .1, 0.05;...
          0, .1, 0.05]';
end
if nargin<2      
       RPos = [ 0, -.1, 0.05;...%EDITED
           0, -.1, 0.05;...
           0, -.1, 0.05;...
           0, -.1, 0.05;...
           0, -.1, 0.05;...
           0, -.1, 0.05;...
           0, -.1, 0.05;...
           0, -.1, 0.05;]';
 end
 if nargin<3
    z=.3
    COMPos = [0.02,0,z;
              0.02,0,z;
              0.02,-.015,.35;
              0.02,-.015,.35;
              0.02,-.0,z;
              0.02,0,z;
              0.02,0,z;
              0.02,0,z;]'; %EDITED
          
end
if nargin<4
    Tol = [.0002,.0002,.0002]';
end

options.floating = true;
legs = RigidBodyManipulator('urdf/Legs.urdf',options);
nq = legs.getNumPositions(); %number of DOF

pelvis = legs.findLinkId('pelvis');
l_foot = legs.findLinkId('l_foot');
r_foot = legs.findLinkId('r_foot');
l_toe = legs.findLinkId('l_foot');
r_toe = legs.findLinkId('r_foot');

Time = 2*10;
Stages = 8;

Allcons = cell(0,1);

Allcons{end+1} = WorldEulerConstraint(legs,pelvis,[nan;0;0]-Tol,[nan;0;0]+Tol,[0,Time]);
Allcons{end+1} = WorldEulerConstraint(legs,l_foot,[pi/2;0;nan]-Tol,[pi/2;0;nan]+Tol,[0,Time]);
Allcons{end+1} = WorldEulerConstraint(legs,r_foot,[pi/2;0;nan]-Tol,[pi/2;0;nan]+Tol,[0,Time]);
Allcons{end+1} = WorldEulerConstraint(legs,l_toe,[pi/2;0;nan]-Tol,[pi/2;0;nan]+Tol,[0,Time]);
Allcons{end+1} = WorldEulerConstraint(legs,r_toe,[pi/2;0;nan]-Tol,[pi/2;0;nan]+Tol,[0,Time]);

q_nom = zeros(nq,1);
q_nom(7:16) = [0 -.5 1.5 -1 0 0 0 .5 1.5 -1]';
for i = 1:Stages-1
    Allcons{6} = WorldPositionConstraint(legs,l_foot,[0;0;0],LPos(:,i)-Tol,LPos(:,i)+Tol,[(i-1)*Time/Stages,(i-1)*Time/Stages]);
    Allcons{7} = WorldPositionConstraint(legs,r_foot,[0;0;0],RPos(:,i)-Tol,RPos(:,i)+Tol,[(i-1)*Time/Stages,(i-1)*Time/Stages]);
    Allcons{8} = WorldCoMConstraint(legs,COMPos(:,i)-Tol,COMPos(:,i)+Tol,[(i-1)*Time/Stages,(i-1)*Time/Stages]);
i=i+1;
    Allcons{9} = WorldPositionConstraint(legs,l_foot,[0;0;0],LPos(:,i)-Tol,LPos(:,i)+Tol,[(i-1)*Time/Stages,(i-1)*Time/Stages]);
    Allcons{10} = WorldPositionConstraint(legs,r_foot,[0;0;0],RPos(:,i)-Tol,RPos(:,i)+Tol,[(i-1)*Time/Stages,(i-1)*Time/Stages]);
    Allcons{11} = WorldCoMConstraint(legs,COMPos(:,i)-Tol,COMPos(:,i)+Tol,[(i-1)*Time/Stages,(i-1)*Time/Stages]);

N = 2;
ikproblem = InverseKinematics(legs,q_nom,Allcons{:});
[q_end_nom,F,info,infeasible_cnstr_ik] = ikproblem.solve(q_nom);
q_nom = q_end_nom;

    ikproblem = InverseKinematics(legs,q_nom,Allcons{:});
    [q_end_nom,F,info,infeasible_cnstr_ik] = ikproblem.solve(q_nom);
    qtraj_guess = PPTrajectory(foh([(i-2)*Time/Stages (i-1)*Time/Stages],[q_nom, q_end_nom])); %more complex then this
    t_vec = linspace((i-2)*Time/Stages,(i-1)*Time/Stages,N);

    % do IK

    ikproblem = InverseKinematicsTrajectory(legs,t_vec,qtraj_guess,false,q_nom,Allcons{:});

    [xtraj,F,info,infeasible_cnstr_ik] = ikproblem.solve(qtraj_guess);

    q_end = xtraj.eval(xtraj.tspan(end));
    
    q_nom=q_end(1:18,1);
    outputTrajTemp{i-1} = xtraj;
   % if info > 10
   %   error('IK fail snopt_info: %d\n', info);
   % end
end

i=2;
while i<Stages
    outputTrajTemp{1} = outputTrajTemp{1}.append(outputTrajTemp{i});
    i=i+1;
end
outputTraj = outputTrajTemp{1};
v = legs.constructVisualizer();
%v.playback(outputTraj,struct('slider',true));