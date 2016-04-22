classdef Caminante < TimeSteppingRigidBodyManipulator & Biped
    methods
        function obj=Caminante(urdf,options)
            typecheck(urdf,'char');

            if nargin < 2
            options = struct();
            end
            if ~isfield(options,'dt')
            options.dt = 0.001;
            end
            if ~isfield(options,'floating')
            options.floating = true;
            end
            if ~isfield(options,'terrain')
            options.terrain = RigidBodyFlatTerrain;
            end
            if ~isfield(options,'external_force')
            options.external_force = [];
            end
            
            w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');

            obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
            obj = obj@Biped('r_foot_sole', 'l_foot_sole', 'r_foot_front', 'l_foot_front');
            warning(w);
            
            % Add a force on a specified link if we want!
            if ~isempty(options.external_force)
                % For compile purposes, record that we have external force applied to a link
                % (this affects input frames)
                obj.external_force = findLinkId(obj,options.external_force);
                options_ef.weld_to_link = obj.external_force;
                obj = obj.addRobotFromURDF(fullfile(getDrakePath,'util','three_dof_force.urdf'), ...
                    [0; 0; 0], [0; 0; 0], options_ef);
            end
            
            % could also do fixed point search here
            obj = obj.setInitialState(obj.resolveConstraints(zeros(obj.getNumStates(),1)));
            
            lastwarn = warning('off', 'Drake:RigidBodySupportState:NoSupportSurface');
            obj.left_full_support = RigidBodySupportState(obj,obj.foot_body_id.left);
            obj.left_toe_support = RigidBodySupportState(obj,obj.foot_body_id.left,struct('contact_groups',{{'toe'}}));
            obj.right_full_support = RigidBodySupportState(obj,obj.foot_body_id.right);
            obj.right_toe_support = RigidBodySupportState(obj,obj.foot_body_id.right,struct('contact_groups',{{'toe'}}));
            obj.left_full_right_full_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right]);
            obj.left_toe_right_full_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right],struct('contact_groups',{{{'toe'},{'heel','toe'}}}));
            obj.left_full_right_toe_support = RigidBodySupportState(obj,[obj.foot_body_id.left,obj.foot_body_id.right],struct('contact_groups',{{{'heel','toe'},{'toe'}}}));
            warning(lastwarn);
            
        end
        
        function obj = compile(obj)
            obj = compile@TimeSteppingRigidBodyManipulator(obj);

            % Construct state vector itself -- start by replacing the
            % caminantePosition and caminanteVelocity frames with a single
            % larger state frame
            if (strcmp(obj.manip.getStateFrame().getFrameByNum(1).name, 'LegsPosition'))
                caminante_state_frame = caminanteFrames.CaminanteState(obj);
            else
                caminante_state_frame = obj.manip.getStateFrame();
                caminante_state_frame = replaceFrameNum(caminante_state_frame,1,caminanteFrames.CaminanteState(obj));
            end
            
            tsmanip_state_frame = obj.getStateFrame();
            if tsmanip_state_frame.dim>caminante_state_frame.dim
                tsmanip_state_frame.frame{1} = caminante_state_frame;
                state_frame = tsmanip_state_frame;
            else
                state_frame = caminante_state_frame;
            end
            obj.manip = obj.manip.setStateFrame(caminante_state_frame);
            obj = obj.setStateFrame(state_frame);
            
            if (obj.external_force > 0)
                input_frame = getInputFrame(obj);
                input_frame = replaceFrameNum(input_frame,1,caminanteFrames.CaminanteInput(obj));
            else
                input_frame = caminanteFrames.CaminanteInput(obj);
            end
            
            obj = obj.setInputFrame(input_frame);
            obj.manip = obj.manip.setInputFrame(input_frame);
        
            % Construct output frame, which comes from state plus sensor
            % info
            caminante_output_frame = caminante_state_frame;
            if (~isempty(obj.manip.sensor))
                for i=1:length(obj.manip.sensor)
                    % If it's not a full state feedback sensor (we have already
                    % got the state for that above in the state frame
                    if (~isa(obj.manip.sensor{i}, 'FullStateFeedbackSensor'))
                        if (isa(caminante_output_frame, 'MultiCoordinateFrame'))
                          caminante_output_frame = caminante_output_frame.appendFrame(obj.manip.sensor{i}.constructFrame(obj.manip));
                        else
                          caminante_output_frame = MultiCoordinateFrame({caminante_output_frame, obj.manip.sensor{i}.constructFrame(obj.manip)});
                        end
                    end
                end
            end
        
            % The output function of a TSRBM appends the TS sensors to the
            % output of the RBM. So get ready for that:
            output_frame = caminante_output_frame;
            if (~isempty(obj.sensor))
                for i=1:length(obj.sensor)
                    if (~isa(obj.sensor{i}, 'FullStateFeedbackSensor'))
                        if (isa(output_frame, 'MultiCoordinateFrame'))
                            output_frame = output_frame.appendFrame(obj.sensor{i}.constructFrame(obj));
                        else
                            output_frame = MultiCoordinateFrame({output_frame, obj.sensor{i}.constructFrame(obj)});
                        end
                    end
                end
            end
            
            if ~isequal_modulo_transforms(caminante_output_frame,getOutputFrame(obj.manip))
                obj.manip = obj.manip.setNumOutputs(caminante_output_frame.dim);
                obj.manip = obj.manip.setOutputFrame(caminante_output_frame);
            end

            if ~isequal_modulo_transforms(output_frame,getOutputFrame(obj))
                obj = obj.setNumOutputs(output_frame.dim);
                obj = obj.setOutputFrame(output_frame);
            end
        end
        
        function obj = setInitialState(obj,x0)
            if isa(x0,'Point')
                obj.x0 = double(x0); %.inFrame(obj.getStateFrame));
            else
                typecheck(x0,'double');
                sizecheck(x0,obj.getNumStates());
                obj.x0 = x0;
            end
        end
        
        function x0 = getInitialState(obj)
            x0 = obj.x0;
        end
        
        function weights = getFootstepOptimizationWeights(obj)
        % Return a reasonable set of default weights for the footstep planner
        % optimization. The weights describe the following quantities:
        % 'relative': the contribution to the cost function of the
        %             displacement from one step to the next
        % 'relative_final': the cost contribution of the displacement of the
        %                   displacement of the very last step (this can be
        %                   larger than the normal 'relative' cost in
        %                   order to encourage the feet to be close together
        %                   at the end of a plan)
        % 'goal': the cost contribution on the distances from the last two
        %         footsteps to their respective goal poses.
        % Each weight is a 6 element vector, describing the weights on
        % [x, y, z, roll, pitch, yaw]

            weights = struct('relative', [1;1;1;0;0;0.5],...
                           'relative_final', [10;10;10;0;0;2],...
                           'goal', [100;100;0;0;0;10]);
        end
        
        function prop_cache = getRobotPropertyCache(obj)
            % Functions like findLinkId, getTerrainContactPoints, etc. can be too slow to call
            % in the inner loop of our controller or planner, so we cache some useful information
            % at setup time. 
            prop_cache = struct('contact_groups', [],...
                                'body_ids', struct(),...
                                'position_indices', struct(),...
                                'actuated_indices', [],...
                                'nq', 0,...
                                'nv', 0,...
                                'num_bodies', 0);

            % getTerrainContactPoints is pretty expensive, so we'll just call it
            % for all the bodies and cache the results
            nbod = length(obj.getManipulator().body);
            contact_group_cache = cell(1, nbod);
            for j = 1:nbod
                contact_group_cache{j} = struct();
                for f = 1:length(obj.getBody(j).collision_geometry_group_names)
                    name = obj.getBody(j).collision_geometry_group_names{f};
                    if obj.getBody(j).robotnum == 1
                        contact_group_cache{j}.(name) = obj.getBody(j).getTerrainContactPoints(name);
                    end
                end
            end

            prop_cache.contact_groups = contact_group_cache;

            prop_cache.nq = obj.getNumPositions();
            prop_cache.nv = obj.getNumVelocities();
            prop_cache.num_bodies = length(obj.getManipulator().body);

            for b = {'pelvis', 'r_foot', 'l_foot'}
                prop_cache.body_ids.(b{1}) = obj.findLinkId(b{1});
            end

            %!! What should this really be?
            %for j = {'Body_to_Left_Hip', 'Left_Hip_to_Top_Left_Leg', 'Top_Left_Leg_to_Bottom_Left_Leg', 'Bottom_Left_Leg_to_Left_Ankle', 'Body_to_Right_Hip', 'Right_Hip_to_Top_Right_Leg','Top_Right_Leg_to_Bottom_Right_Leg','Bottom_Right_Leg_to_Right_Ankle'}
            for j = {'l_hip','l_knee','l_ankle','r_hip','r_knee','r_ankle'}
                prop_cache.position_indices.(j{1}) = obj.findPositionIndices(j{1});
            end

            prop_cache.actuated_indices = obj.getActuatedJoints();
        end
    end
        
    properties (SetAccess = protected, GetAccess = public)
        x0
        % preconstructing these for efficiency
        left_full_support
        left_toe_support
        right_full_support
        right_toe_support
        left_full_right_full_support
        left_toe_right_full_support
        left_full_right_toe_support
        caminante_version = [];
        external_force = 0; % if nonzero, body id where force is being exerted
    end

    properties
        fixed_point_file = fullfile(pwd, 'data', 'caminante_fp.mat');
        %fixed_point_file = fullfile(pwd, 'data', 'caminante_fp_bent.mat');
        %!! go through these in more detail
        default_footstep_params = struct('nom_forward_step', 0.05,... % m
                                         'max_forward_step', 0.07,...% m
                                         'max_backward_step', 0.07,...% m
                                         'max_step_width', 0.30,...% m
                                         'min_step_width', 0.20,...% m
                                         'nom_step_width', 0.25,...% m
                                         'max_outward_angle', 0.01,... % rad %!! can we set to 0?
                                         'max_inward_angle', 0.01,... % rad
                                         'nom_upward_step', 0.05,... % m
                                         'nom_downward_step', 0.05,...% m
                                         'max_num_steps', 10,...
                                         'min_num_steps', 1,...
                                         'leading_foot', 1); % 0: left, 1: right
        default_walking_params = struct('step_speed', 0.3,... % speed of the swing foot (m/s)
                                        'step_height', 0.02,... % approximate clearance over terrain (m)
                                        'drake_min_hold_time', 0.7,... % minimum time in double support (s)
                                        'drake_instep_shift', 0.0,... % Distance to shift ZMP trajectory inward toward the instep from the center of the foot (m)
                                        'mu', 1.0,... % friction coefficient
                                        'constrain_full_foot_pose', true,... % whether to constrain the swing foot roll and pitch
                                        'pelvis_height_above_foot_sole', 0.5,... % default pelvis height when walking
                                        'support_contact_groups', {{'heel', 'toe'}},... % which contact groups are available for support when walking
                                        'prevent_swing_undershoot', false,... % prevent the first phase of the swing from going backwards while moving to the first knot point
                                        'prevent_swing_overshoot', false,... % prevent the final phase of the swing from moving forward of the last knot point
                                        'nominal_LIP_COM_height', 0.50); % nominal height used to construct D_ls for our linear inverted pendulum model
        pelvis_name = 'pelvis';
        r_foot_name = 'r_foot';
        l_foot_name = 'l_foot';
        r_knee_name = 'r_knee';
        l_knee_name = 'l_knee';
        l_akx_name = 'l_ankle_roll';
        r_akx_name = 'r_ankle_roll';
        r_aky_name = 'r_ankle_flex';
        l_aky_name = 'l_ankle_flex';
    end
end