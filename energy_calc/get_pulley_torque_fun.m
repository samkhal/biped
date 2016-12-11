% Sam Khalandovsky
% generate the torque function for a given pulley, defined by alpha offsets
% optionally, draw the pulley

%% input
% See generate_pulley for input info
% px_1 is the x-intercept of the first cable path, always negative

%% output
% fun: angle->torque function that the pulley produces
% c: a vector of inequality constraints for optimization (c <= 0)

function [fun, c, pulley_ps] = get_pulley_torque_fun(anchor_dist, anchor_offset, theta_max, spring_max, k_spring, px_1, alpha_dists, draw)
    persistent cross2d
    if isempty(cross2d)
        %get magnitude of cross product from two vectors
        cross2d = @(a,b) subsref(cross(horzcat(a,0),horzcat(b,0)),substruct('()',{3}));
    end
    
    last_point = [px_1, 0];
    last_f_spring = spring_max;
    
    torques = zeros(1,length(alpha_dists)+1);
    torques(1) = last_f_spring * abs(px_1)*anchor_dist/hypot(anchor_dist,px_1+anchor_offset);    
    
    % define virtual anchor points as pulley is unwrapped
    anchor_r = hypot(anchor_dist, anchor_offset);
    anchor_theta = atan(anchor_offset/anchor_dist);
    thetas = linspace(0, theta_max, length(alpha_dists)+1);
    anchor_ps(:,1) = -anchor_r*sin(thetas+anchor_theta);
    anchor_ps(:,2) = -anchor_r*cos(thetas+anchor_theta);
    
    if draw
        pulley_ps = zeros(length(alpha_dists),2);
        f_springs = zeros(length(alpha_dists),1);
    end
    
    % track min and max radii
    max_pulley_r = 0;
    min_pulley_lever = Inf;
    worst_bottom_right_quadrant = 0;
    min_free_len = Inf;
    min_angle_to_origin = Inf; 
    max_f_spring = 0;
    min_f_spring = Inf;
    
    last_anchor_to_pulley = last_point - anchor_ps(1,:);
    last_free_len = norm(last_anchor_to_pulley);
    
    for i = 2:length(alpha_dists)+1
        new_point = last_point + last_anchor_to_pulley/last_free_len * alpha_dists(i-1);
        anchor_to_pulley = new_point-anchor_ps(i,:);
        
        free_len = norm(anchor_to_pulley);
        prev_len = alpha_dists(i-1) + last_free_len;
        x_diff = free_len - prev_len;
        
        %new radius is distance from pull line to origin
        lever_arm = abs(cross2d(anchor_to_pulley,-anchor_ps(i,:)))/free_len;

        f_spring = max(0, last_f_spring + k_spring*x_diff); %force can't go below 0
        torques(i) = f_spring * lever_arm;
        
        % Store values for constraints
        max_f_spring = max(max_f_spring, f_spring);
        min_f_spring = min(min_f_spring, f_spring);
        r = norm(new_point);
        max_pulley_r = max(max_pulley_r, r);
        min_pulley_lever = min(min_pulley_lever, lever_arm);
        min_free_len = min(min_free_len, free_len);
        min_angle_to_origin = min(min_angle_to_origin,cross2d(-anchor_ps(i,:),anchor_to_pulley));        
        
        if new_point(1)>0 && new_point(2)<0 %bottom right quadrant? forbidden
            worst_bottom_right_quadrant = max(worst_bottom_right_quadrant, new_point(1) - new_point(2));
        end
        
        % Store values for drawing
        if draw
            pulley_ps(i-1,:) = new_point;
            f_springs(i-1) = f_spring;
        end
        
        % Store calculated values for next loop
        last_f_spring = f_spring;
        last_point = new_point;
        last_anchor_to_pulley = anchor_to_pulley;
        last_free_len = free_len;
    end
     
    if draw
        figure(100);
        cla reset;
        axis equal
        hold on
        plot(anchor_ps(:,1),anchor_ps(:,2),'b');
        connect_origin = false;
        if connect_origin
            plot([pulley_ps(:,1); 0; pulley_ps(1,1)],[pulley_ps(:,2); 0; pulley_ps(1,2)],'r.-'); 
        else
            plot(pulley_ps(:,1),pulley_ps(:,2),'r.-'); 
        end
        plot([0 0 -anchor_offset],[0 -anchor_dist -anchor_dist], 'k', 'LineWidth',2)
        plot(px_1,0,'k.');
        for i = [1 floor(size(pulley_ps,1)/2) size(pulley_ps,1)]
            plot([pulley_ps(i,1) anchor_ps(i,1)],[pulley_ps(i,2) anchor_ps(i,2)],'k--')
        end
        pad = diff(ylim)*0.1;
        ylim(ylim + [-pad pad]);
        xlabel('x (m)');
        ylabel('y (m)');
        
        figure(90);
        cla reset;
        axis equal
        
        plot(thetas(1:end-1), imp(f_springs,'N')); %TODO: why diff len?
        xlabel('theta (rad)');
        ylabel('Force (lbf)')
        
        plot(thetas, imp(torques,'N*m'), 'LineWidth',2);
        title('Torque profile');
        xlabel('angle (rad)');
        ylabel('Torque (lbf*in)');
    end
    
    fun = @(angle) interp1(thetas,torques,angle);
    
    global MAX_RADIUS_LIM MIN_RADIUS_LIM MIN_SPRING_LEN MIN_SPRING_F
    if nargout>1
        % Calculate min/max radius constraints
    
        %c(1,1) = max_pulley_r - MAX_RADIUS_LIM;
        
        c(1,1) = (MIN_RADIUS_LIM - min_pulley_lever);
        %c(2,1) = 0;

        % Prevent points from entering +x/-y quadrant
        %c(3,1) = worst_bottom_right_quadrant;
        %c(3,1) = 0;

        % Prevent angle between origin->anchor->pulley from becoming negative
        c(2,1) = -min_angle_to_origin;

        % Restrict minimum spring size
        max_spring_extension = max_f_spring/k_spring;
        min_spring_len_upper_bound = min_free_len - max_spring_extension;
        %c(4,1) = MIN_SPRING_LEN - min_spring_len_upper_bound;
        %c(4,1) = 0;

        % Restrict minimum spring load
        %c(5,1) = MIN_SPRING_F - min_f_spring; %minimum f from pulley must be greater than initial spring tension
    end
        
    if draw
        min_spring_clearance = imp(min_free_len,'m') % minimum value of free length. Achieved when spring is maximally extended
        max_spring_extension_in = imp(max_f_spring/k_spring, 'm')
        min_spring_len_upper_bound_in = imp(min_spring_len_upper_bound, 'm')
        max_f_spring_lbf = imp(max_f_spring, 'N')
        min_f_spring_lbf = imp(min_f_spring, 'N')
        k_lbf_in = imp(k_spring,'N/m')
    end
    
    
    
    
        