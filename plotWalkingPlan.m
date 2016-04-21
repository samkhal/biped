% Visualize walking plans

load CaminanteTwoStepPlan
%load AtlasTwoStepPlan

dt = 0.1;
t = 0:dt:walking_plan_data.settings.zmptraj(1).tspan(2);
zmptrajXY = walking_plan_data.settings.zmptraj(1).eval(t);
zmptraj_extra = walking_plan_data.settings.zmptraj(2);
zmptraj_extra = zmptraj_extra.eval(t);

comtrajXY = walking_plan_data.settings.comtraj(1).eval(t);
comtraj_extra = walking_plan_data.settings.comtraj(2);
comtraj_extra = comtraj_extra.eval(t);

r_front_traj = PPTrajectory(walking_plan_data.settings.body_motions(1).getPP());
l_front_traj = PPTrajectory(walking_plan_data.settings.body_motions(2).getPP());
pelvis_traj = PPTrajectory(walking_plan_data.settings.body_motions(3).getPP());

r_front_traj = r_front_traj.eval(t);
l_front_traj = l_front_traj.eval(t);
pelvis_traj = pelvis_traj.eval(t);

foot_pos = [footstep_plan.footsteps.pos];

figure;
hold on;
plot3(r_front_traj(1,:),r_front_traj(2,:),r_front_traj(3,:))
plot3(l_front_traj(1,:),l_front_traj(2,:),l_front_traj(3,:))
plot3(pelvis_traj(1,:),pelvis_traj(2,:),pelvis_traj(3,:))

figure;
hold on;
plot(zmptrajXY(1,:),zmptrajXY(2,:))
plot(comtraj(1,:),comtraj(2,:))
plot(foot_pos(1,:),foot_pos(2,:),'*')
legend('ZMP','COM','Footsteps','Location','south')
title('ZMP-COM XY')

figure;
hold on
plot(t,zmptraj_extra)
plot(t,comtraj_extra)
title('ZMP-COM extra vs time')
