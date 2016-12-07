% Sam Khalandovsky
% Try to optimize the constant-radius pulley parameters. Assuming pi/2 as
% the angle range.

%% Inputs
% traj: trajectory to optimize for
% initial_pulley_vals: vector of parameters to optimize

%% Outputs
% pulley_vals: vector of optimized parameters
% fraction_saved: fraction of energy saved by this pulley

%params: f_min, r, k

function [pulley_vals, cost] = optimize_pulley_constant_radius(traj, xlow, xupp, spring_max)
    % TODO constants separate
    knee_motor_consts = [680, 14, .527];
    gearbox = 111;
    joint = 'r_leg_kny';
    joints_velocity = [traj.vel.(joint)];
    joints_torque = [traj.torque.(joint)];
    joints_angle = [traj.pos.(joint)];
    dt = traj.times(2)-traj.times(1);

    function cost = cost_function(p_vals)
        f_min = p_vals(1);
        r = p_vals(2);
        k = p_vals(3);
        f_max = f_min + k*pi/2*r;
        t_fun = @(angle) r*(f_min*(angle/(pi/2)) + f_max*(1 - angle/(pi/2)));
        
        additional_torque = t_fun(pi/2 - joints_angle);
        
        power = zeros(1,size(traj.torque,2));
        
        %for each motor, for each time, add up energy with and without spring 
        power(:) = electrical_power([knee_motor_consts(2), knee_motor_consts(3)],gearbox.*joints_velocity, 1/gearbox.*abs(joints_torque+additional_torque(:)'), 0);
        cost = sum(sum(power))*dt;
    end

    function [c_ineq, c_eq] = constraint(p_vals)
        f_min = p_vals(1);
        r = p_vals(2);
        k = p_vals(3);
        f_max = f_min + k*pi/2*r;
        
        c_ineq = f_max - spring_max;
        c_eq = [];
    end

    
    
    if nargin<2
        len = 3; xlow=zeros(len,1); xupp = xlow;
        xlow(1)=si(14,'lbf');xupp(1)=si(55,'lbf'); % f_min
        xlow(2)=si(1,'in');xupp(2)=si(1,'in'); % r
        xlow(3)=si(26,'lbf/in');xupp(3)=si(26,'lbf/in'); % k
    end
    
    % initialize with max torque of 4.5 N*m
    f_max_init = 4.5;
    f_min_init = si(10,'lbf');
    r_init = si(0.5,'in');
    k_init = (f_max_init-f_min_init)/(pi/2 * r_init);
    initial_pulley_vals = [f_min_init; r_init; k_init]; 
    initial_pulley_vals = (xupp - xlow)/2;
        
    [pulley_vals, cost, exitflag, output] = snsolve(@cost_function, initial_pulley_vals, [], [], [], [], xlow, xupp, @constraint);
    output
    f_min_lbf = imp(pulley_vals(1),'N')
    r_in = imp(pulley_vals(2),'m')
    k_imp = imp(pulley_vals(3),'N/m')
    f_max_lbf = f_min_lbf + k_imp*pi/2*r_in
    

    power_wo_pulley = electrical_power([knee_motor_consts(2), knee_motor_consts(3)], gearbox*joints_velocity, 1/gearbox*abs(joints_torque), 0); %power, W   
    energy_wo_pulley = sum(power_wo_pulley)*dt;
    
    fraction_saved = 1-cost/energy_wo_pulley
    
end