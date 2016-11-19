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
    
    function cost = cost_function(p_vals)
        % calculate torque function
        t_fun = get_pulley_torque_fun(p_vals(1),p_vals(2),p_vals(3),p_vals(4),p_vals(5),p_vals(6),p_vals(7:end), 0);
        additional_torque = t_fun(pi/2 - joints_angle);
        
        power = zeros(1,size(traj.torque,2));
        
        %for each motor, for each time, add up energy with and without spring 
        power(:) = electrical_power([knee_motor_consts(2), knee_motor_consts(3)],gearbox.*joints_velocity, 1/gearbox.*abs(joints_torque+additional_torque(:)'), 0);
        cost = sum(sum(power))*dt;
    end

    % Set up optimization bounds
    len = length(initial_pulley_vals);
    xlow = zeros(len,1); xupp = zeros(len,1);
    xlow(1)=0; xupp(1)=0.3048; %anchor_dist
    xlow(2)=0; xupp(2)=0.3048; %anchor_offset
    xlow(3)=pi/2; xupp(3)=pi/2; %theta_max
    xlow(4)=10; xupp(4)=2000; %spring_max
    xlow(5)=10; xupp(5)=90000; %spring_k
    xlow(6)=-0.3048; xupp(6)=-.01; %px_1
    xlow(7)=-0.3048; xupp(7)=0.3048; % first alpha_dist
    xlow(8:len)=0; xupp(8:len)=0.01; % all other alpha_dist
    
    % Set constraint on sum of alpha_dists
    A = zeros(1,len); A(8:end) = 1;
    b = pi/2*.3; %max length is quarter circle of 1 foot radius
    
    snprint('snsolve.out');
    [pulley_vals, cost] = snsolve(@cost_function, initial_pulley_vals, A, b, [], [], xlow, xupp);
        
    power_wo_pulley = electrical_power([knee_motor_consts(2), knee_motor_consts(3)], gearbox*joints_velocity, 1/gearbox*abs(joints_torque), 0); %power, W   
    energy_wo_pulley = sum(power_wo_pulley)*dt;
    
    fraction_saved = 1-cost/energy_wo_pulley;
    
end