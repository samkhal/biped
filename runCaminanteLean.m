function runCaminanteLean(LPos, RPos, COMPos, Tol)
%% Ari Goodman
%% 3/25/2016
%TEMPORARILLY KINDA SORTA NOT WORKING
% Hit Run with debugger and analyze each section. IT KINDA WALKS
% Errors: feet slide when footprints should be stationary (COM movement
% should be different)
% Should use hybrid system or something because the total trajectory doesnt
% work with times as originally thought

if nargin<1
  LPos = [0, .1, 0.03;...
          0, .1, 0.03;...
          .125,.1, 0.15;...
          .25, .1, 0.03;...
          .25, .1, 0.03;...
          .25, .1, 0.03;...
          .25, .1, 0.03;...
          .25, .1, 0.03;]';
end
if nargin<2
  RPos = [ 0, -.1, 0.06;...
           0, -.1, 0.06;...
           0, -.1, 0.06;...
           0, -.1, 0.06;...
           0, -.1, 0.06;...
          .125,-.1, 0.15;...
          .25, -.1, 0.06;...
          .25, -.1, 0.06;]';
 end
 if nargin<3
    COMPos = [0,0,nan;
              0,-.1,nan;
              .05,-.1,nan;
              nan,-.1,nan;
              .2,.1,nan;
              .2,.1,nan;
              .2,.1,nan;
              .25,0,nan;]';
end
if nargin<4
    Tol = [.02,.02,.005]';
end

options.floating = true;
legs = RigidBodyManipulator('urdf2/Legs.urdf',options);
nq = legs.getNumPositions(); %number of DOF

pelvis = legs.findLinkId('pelvis');
l_foot = legs.findLinkId('l_foot');
r_foot = legs.findLinkId('r_foot');
l_toe = legs.findLinkId('l_foot');
r_toe = legs.findLinkId('r_foot');

Time = 10;
Stages = 8;

Allcons = cell(0,1);

Allcons{end+1} = WorldEulerConstraint(legs,pelvis,-Tol,Tol,[0,Time]);
Allcons{end+1} = WorldEulerConstraint(legs,l_foot,[pi/2;0;nan]-Tol,[pi/2;0;nan]+Tol,[0,Time]);
Allcons{end+1} = WorldEulerConstraint(legs,r_foot,[pi/2;0;nan]-Tol,[pi/2;0;nan]+Tol,[0,Time]);
Allcons{end+1} = WorldEulerConstraint(legs,l_toe,[pi/2;0;nan]-Tol,[pi/2;0;nan]+Tol,[0,Time]);
Allcons{end+1} = WorldEulerConstraint(legs,r_toe,[pi/2;0;nan]-Tol,[pi/2;0;nan]+Tol,[0,Time]);

for potato=1:Stages

    q_nom = zeros(nq,1);
    if potato ~= 1
        for i=potato-1
            Allcons{6} = WorldPositionConstraint(legs,l_foot,[0;0;0],LPos(:,i)-Tol,LPos(:,i)+Tol,[Time,Time]);
            Allcons{7} = WorldPositionConstraint(legs,r_foot,[0;0;0],RPos(:,i)-Tol,RPos(:,i)+Tol,[Time,Time]);
            Allcons{8} = WorldCoMConstraint(legs,COMPos(:,i)-Tol,COMPos(:,i)+Tol,[Time,Time]);
        T = Time;
        N = 2;
        v = legs.constructVisualizer();
        ikproblem = InverseKinematics(legs,q_nom,Allcons{:});
        [q_end_nom,F,info,infeasible_cnstr_ik] = ikproblem.solve(q_nom);
        q_nom = q_end_nom;
        end
    end
    for i=potato
        Allcons{6} = WorldPositionConstraint(legs,l_foot,[0;0;0],LPos(:,i)-Tol,LPos(:,i)+Tol,[Time,Time]);
        Allcons{7} = WorldPositionConstraint(legs,r_foot,[0;0;0],RPos(:,i)-Tol,RPos(:,i)+Tol,[Time,Time]);
        Allcons{8} = WorldCoMConstraint(legs,COMPos(:,i)-Tol,COMPos(:,i)+Tol,[Time,Time]);
    end

    T = Time;
    N = 2;
    v = legs.constructVisualizer();

    ikproblem = InverseKinematics(legs,q_nom,Allcons{:});
    [q_end_nom,F,info,infeasible_cnstr_ik] = ikproblem.solve(q_nom);
    qtraj_guess = PPTrajectory(foh([0 T],[q_nom, q_end_nom])); %more complex then this

    t_vec = linspace(0,T,N);

    % do IK

    ikproblem = InverseKinematicsTrajectory(legs,t_vec,qtraj_guess,false,q_nom,Allcons{:});

    [xtraj,F,info,infeasible_cnstr_ik] = ikproblem.solve(qtraj_guess);

    q_end = xtraj.eval(xtraj.tspan(end));
    % do visualize

    v.playback(xtraj,struct('slider',true));

    if info > 10
      error('IK fail snopt_info: %d\n', info);
    end
end
end