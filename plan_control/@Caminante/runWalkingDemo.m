function [ytraj, com, walking_plan_data, footstep_plan] = runWalkingDemo(obj,walking_options)
% Walk to a target goal, and transition to standing when the goal is
% reached.


checkDependency('gurobi');
checkDependency('lcmgl');

% Fmincon throws various errors, use snopt if possible
checkDependency('snopt');

if nargin < 2; walking_options = struct(); end;

walking_options = applyDefaults(walking_options, struct('initial_pose', [],...
                                                        'navgoal', [1;0;0;0;0;0],...
                                                        'max_num_steps', 10,...
                                                        'rms_com_tolerance', 0.0051));


walking_options = applyDefaults(walking_options, obj.default_footstep_params);
walking_options = applyDefaults(walking_options, obj.default_walking_params);

% set initial state to fixed point
load(obj.fixed_point_file, 'xstar');
if ~isempty(walking_options.initial_pose), xstar(1:6) = walking_options.initial_pose; end
xstar = obj.resolveConstraints(xstar);
obj = obj.setInitialState(xstar);

v = obj.constructVisualizer;
v.display_dt = 0.01;

nq = getNumPositions(obj);

x0 = xstar;

% Find the initial positions of the feet
R=rotz(walking_options.navgoal(6));

rfoot_navgoal = walking_options.navgoal;
lfoot_navgoal = walking_options.navgoal;

%!!! hardcoded widths replaced with calculation from nom_step_width
half_step_width = walking_options.nom_step_width/2;
rfoot_navgoal(1:3) = rfoot_navgoal(1:3) + R*[0;-half_step_width;0];
lfoot_navgoal(1:3) = lfoot_navgoal(1:3) + R*[0;half_step_width;0];


% Plan footsteps to the goal
goal_pos = struct('right', rfoot_navgoal, 'left', lfoot_navgoal);
footstep_plan = obj.planFootsteps(x0(1:nq), goal_pos, [], struct('method_handle',@footstepPlanner.humanoids2014,'step_params', walking_options));
%footstep_plan = obj.planFootsteps(x0(1:nq), goal_pos, [], struct('step_params', walking_options));
for j = 1:length(footstep_plan.footsteps)
  footstep_plan.footsteps(j).walking_params = walking_options;
end

% Show the footstep plan
v.draw(0, xstar);
if isa(v, 'BotVisualizer')
 lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'footstep_plan');
  footstep_plan.draw_lcmgl(lcmgl);
  lcmgl.switchBuffers();
else
  figure(25)
  footstep_plan.draw_2d();
end

% Generate a dynamic walking plan with zmp, com, and foot trajectories
walking_plan_data = obj.planWalkingZMP(x0(1:obj.getNumPositions()), footstep_plan);

% Do initial drawing before the simulation
obj.plotWalkingPlan(walking_plan_data, footstep_plan);

% Execute the plan
[ytraj, com, rms_com] = obj.simulateWalking(walking_plan_data);

v.playback(ytraj, struct('slider', true));

% Check COM error

% if ~rangecheck(rms_com, 0, walking_options.rms_com_tolerance);
%   error('Drake:runWalkingDemo:BadCoMTracking', 'Center-of-mass during execution differs substantially from the plan.');
% end



