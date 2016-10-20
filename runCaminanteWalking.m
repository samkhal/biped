
robot_options = struct( 'use_bullet', true,...
                        'terrain', RigidBodyFlatTerrain,...
                        'floating', true,...
                        'ignore_self_collisions', true,...
                        'ignore_friction', true,...
                        'enable_fastqp', false,...
                        'dt', 0.001);

walking_options = struct('navgoal', [1;0;0;0;0;0],...
                         'max_num_steps', 10);    
                                                                                                    
                                                
% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
r = Caminante('urdf/caminante_minimal.urdf',robot_options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);     

r.runWalkingDemo();