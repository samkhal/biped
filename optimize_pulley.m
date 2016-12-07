% Sam Khalandovsky
% Try to optimize the pulley parameters to decrease energy usage using
% nonlinear optimization

%% Inputs
% traj: trajectory to optimize for
% initial_pulley_vals: vector of parameters to optimize

%% Outputs
% pulley_vals: vector of optimized parameters
% fraction_saved: fraction of energy saved by this pulley

function [pulley_vals, fraction_saved] = optimize_pulley(traj, initial_pulley_vals)
    % start with a candidate from knee_pulley
    
    % TODO constants separate
    knee_motor_consts = [680, 14, .527];
    gearbox = 111;
    joint = 'r_leg_kny';
    joints_velocity = [traj.vel.(joint)];
    joints_torque = [traj.torque.(joint)];
    joints_angle = [traj.pos.(joint)];
    dt = traj.times(2)-traj.times(1);
    
    last_p_vals = []; %cache last p_vals to avoid recomputation between cost and constraint function calls
    t_fun = []; %cache function and constraint results
    c = [];
    function cost = cost_function(p_vals)
        if ~isequal(p_vals, last_p_vals)
            % calculate torque function
            [t_fun, c] = get_pulley_torque_fun(p_vals(1),p_vals(2),p_vals(3),p_vals(4),p_vals(5),p_vals(6),p_vals(7:end), 0);
            last_p_vals = p_vals;
        end
        
        additional_torque = t_fun(pi/2 - joints_angle);
        
        power = zeros(1,size(traj.torque,2));
        
        %for each motor, for each time, add up energy with and without spring 
        power(:) = electrical_power([knee_motor_consts(2), knee_motor_consts(3)],gearbox.*joints_velocity, 1/gearbox.*abs(joints_torque+additional_torque(:)'), 0);
        cost = sum(sum(power))*dt;
    end

    function [c_ineq, c_eq] = constraint(p_vals)
        if ~isequal(p_vals, last_p_vals)
            % calculate torque function
            [t_fun, c] = get_pulley_torque_fun(p_vals(1),p_vals(2),p_vals(3),p_vals(4),p_vals(5),p_vals(6),p_vals(7:end), 0);
            last_p_vals = p_vals;
        end
        c_ineq = c;
        c_eq = [];
    end
        
    % Set up optimization bounds
    len = length(initial_pulley_vals);
    xlow = zeros(len,1); xupp = zeros(len,1);
    %xlow(1)=si(1,'in'); xupp(1)=si(8,'in'); %anchor_dist
    xlow(1)=si(7.54,'in'); xupp(1)=si(7.54,'in'); %anchor_dist
    %xlow(2)=si(0,'in'); xupp(2)=si(1,'in'); %anchor_offset
    xlow(2)=si(0.47,'in'); xupp(2)=si(0.47,'in'); %anchor_offset
    xlow(3)=pi/2; xupp(3)=pi/2; %theta_max
    xlow(4)=10; xupp(4)=si(200,'lbf'); %spring_max
    xlow(5)=10; xupp(5)=si(100,'lbf/in'); %spring_k
    
    xlow(6)=si(-1,'ft'); xupp(6)=-.01; %px_1
    xlow(7)=si(-1,'ft'); xupp(7)=si(1,'ft'); % first alpha_dist
    xlow(8:len)=0; xupp(8:len)=0.01; % all other alpha_dist
    
    % Set constraint on sum of alpha_dists
    %A = zeros(1,len); A(8:end) = 1;
    %b = pi/2*.3; %max length is quarter circle of 1 foot radius
    
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    
    global MAX_RADIUS_LIM MIN_RADIUS_LIM MIN_SPRING_LEN MIN_SPRING_F
    %MAX_RADIUS_LIM = 0.05;
    MIN_RADIUS_LIM = si(0.5,'in');
    %MIN_SPRING_LEN = si(1,'in');
    
    define_spring = 1;
    if define_spring
        k = si(93,'lbf/in');
        max_load = si(114,'lbf');
        min_load = si(10, 'lbf'); %incorporated through max displacement constraint
        MIN_SPRING_LEN = si(3.5, 'lbf');
        
        max_displacement = (max_load - min_load)/k;
        
        xlow(4)=min_load; xupp(4)=max_load; %spring_max: force must be within spring bounds
        xlow(5)=k; xupp(5)=k; %spring_k: exact amtch
        
        A = zeros(1,len); A(8:end) = 1; %sum alpha_dists after the first must be below max displacement
        b = max_displacement;
    end
    
    
    
    snprint('snsolve.out');
    [pulley_vals, cost, exitflag, output] = snsolve(@cost_function, initial_pulley_vals, A, b, Aeq, beq, xlow, xupp, @constraint);
    disp(output);
    power_wo_pulley = electrical_power([knee_motor_consts(2), knee_motor_consts(3)], gearbox*joints_velocity, 1/gearbox*abs(joints_torque), 0); %power, W   
    energy_wo_pulley = sum(power_wo_pulley)*dt;
    
    fraction_saved = 1-cost/energy_wo_pulley;
    
end