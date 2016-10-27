
robot_options = struct( 'use_bullet', true,...
                        'terrain', RigidBodyFlatTerrain,...
                        'floating', true,...
                        'ignore_self_collisions', true,...
                        'ignore_friction', true,...
                        'enable_fastqp', false,...
                        'dt', 0.001);
                    


% Calculate navgoal for a turning or straight path
turning_radius = 5;                    
dist = 1;       
goal_yaw = dist/turning_radius;
goal_y = -turning_radius + turning_radius*cos(goal_yaw);
goal_x = turning_radius*sin(goal_yaw);
navgoal_arc = [goal_x;goal_y;0;0;0;-goal_yaw];

navgoal_basic = [dist;0;0;0;0;0];
walking_options = struct('navgoal', navgoal_basic,...
                          'max_num_steps', 10);                
                          
% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
r = Caminante('urdf/caminante_minimal.urdf',robot_options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);     

global torques frame
frame = 1;
seconds = 13;
torques = nan(r.getInputFrame.dim,seconds/robot_options.dt);

[ytraj, com, walking_plan_data, footstep_plan] = r.runWalkingDemo(walking_options);