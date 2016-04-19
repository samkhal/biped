function runCaminanteWalking(robot_options, walking_options)
% Run the new split QP controller, which consists of separate PlanEval
% and InstantaneousQPController objects. The controller will also
% automatically transition to standing when it reaches the end of its walking
% plan.

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin < 1; robot_options = struct(); end;
if nargin < 2; walking_options = struct(); end;

walking_options = applyDefaults(walking_options, struct(...%'initial_pose', [],...
                                                        'navgoal', [0.05;0;0;0;0;0],...
                                                        'max_num_steps', 6,...
                                                        'rms_com_tolerance', 0.0051));

robot_options = applyDefaults(robot_options, struct('use_bullet', true,...
                                                    'terrain', RigidBodyFlatTerrain,...
                                                    'floating', true,...
                                                    'ignore_self_collisions', true,...
                                                    'ignore_friction', true,...
                                                    'enable_fastqp', false,...
                                                    'dt', 0.001));
% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
r = Caminante('urdf/Legs.urdf',robot_options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

%walking_options = applyDefaults(walking_options, struct('initial_pose',getPoseBentLegs(r)));

r.runWalkingDemo(walking_options);