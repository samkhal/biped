%% input
% See generate_pulley for input info
% px_1 is the x-intercept of the first cable path, always negative

function fun = get_pulley_torque_fun(anchor_dist, anchor_offset, theta_max, spring_max, k_spring, px_1, alpha_dists, draw)

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
    end
    
    for i = 2:length(alpha_dists)+1
        last_anchor_to_pulley = last_point-anchor_ps(i-1,:);
        alpha = last_point + last_anchor_to_pulley/norm(last_anchor_to_pulley) * alpha_dists(i-1);
        last_point = alpha;
        free_len = norm(anchor_ps(i,:) - alpha);
        free_len_last = alpha_dists(i-1) + norm(last_anchor_to_pulley);
        x_diff = free_len - free_len_last;
        
        if draw %save points
            pulley_ps(i-1,:) = alpha;
        end

        %new radius is distance from pull line to origin
        cross2d = @(a,b) cross(horzcat(a,0),horzcat(b,0));
        r = norm(cross2d(alpha-anchor_ps(i,:),-anchor_ps(i,:)))/norm(alpha-anchor_ps(i,:));

        f_spring = last_f_spring + k_spring*x_diff;
        torques(i) = f_spring * r;
        
        last_f_spring = f_spring;
    end
    
    if draw
        figure(100);
        cla reset;
        axis equal
        hold on
        plot(anchor_ps(:,1),anchor_ps(:,2),'b');
        plot([pulley_ps(:,1); 0; pulley_ps(1,1)],[pulley_ps(:,2); 0; pulley_ps(1,2)],'r.'); 
        plot([0 0 -anchor_offset],[0 -anchor_dist -anchor_dist], 'k', 'LineWidth',2)
        plot(px_1,0,'k.');
        for i = [1 floor(size(pulley_ps,1)/2) size(pulley_ps,1)]
            plot([pulley_ps(i,1) anchor_ps(i,1)],[pulley_ps(i,2) anchor_ps(i,2)],'k--')
        end
        pad = diff(ylim)*0.1;
        ylim(ylim + [-pad pad]);
    end
    
    fun = @(angle) interp1(thetas,torques,angle);
        