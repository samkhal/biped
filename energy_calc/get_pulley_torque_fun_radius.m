% Generate a torque function for a pulley defined by radii

%% input
% See generate_pulley for input info for most vars
% theta_1: angle between first pulley point and -y axis
% d_theta: angle between consecutive points
% rads: radiuses of each point

function fun = get_pulley_torque_fun_radius(anchor_dist, anchor_offset, theta_max, spring_max, k_spring, theta_1, d_theta, rads, draw)

    pulley_thetas = (0:length(rads)-1) * d_theta + theta_1;
    
    %polar to cartesian conversion. Polar is [theta, radius]. Theta is CW from -X.
    my_pol2cart = @(polar) [-polar(2)*sin(polar(1)) -polar(2)*cos(polar(1))]; 
    
    %get magnitude of cross product from two vectors
    cross2d = @(a,b) subsref(cross(horzcat(a,0),horzcat(b,0)),substruct('()',{3}));
    
    pulley_ps(:,1) = -rads.*sin(pulley_thetas);
    pulley_ps(:,2) = -rads.*cos(pulley_thetas);    
    pulley_diffs = diff(pulley_ps);
    pulley_dists = hypot(pulley_diffs(:,1),pulley_diffs(:,2));

    anchor_init = [-anchor_offset, -anchor_dist];
    r_init = abs(cross2d(pulley_ps(1,:)-anchor_init, -anchor_init))/norm(pulley_ps(1,:)-anchor_init);
    
    torques = zeros(1,length(rads)+1); %torques(i) means the torque when the cable touches points i and i-1
    torques(1) = spring_max * r_init; 
    
    % define virtual anchor points as pulley is unwrapped
    anchor_r = hypot(anchor_dist, anchor_offset);
    anchor_theta_init = atan(anchor_offset/anchor_dist);
    
    anchor_thetas = zeros(1,length(rads)+1);
    anchor_thetas(1) = anchor_theta_init;
    
    last_f_spring = spring_max;
    free_len_last = norm(anchor_init - pulley_ps(1,:));

    if draw
        anchor_ps = zeros(length(rads)+1,2);
        anchor_ps(1,:) = anchor_init;
    end

    for i = 2:length(rads)
        % derivation: pg. 5 sam's notebook
        d = pulley_dists(i-1);
        beta = pi - asin(rads(i)/d * sin(d_theta));
        gamma = asin(rads(i-1)/anchor_r * sin(beta));
        sigma = pi - beta - gamma;
        
        anchor_thetas(i) = pulley_thetas(i-1)-sigma;
        anchor = my_pol2cart([pulley_thetas(i-1)-sigma, anchor_r]);
        free_len = norm(anchor - pulley_ps(i-1,:));
                
        x_diff = free_len - free_len_last;
        
        %new radius is distance from pull line to origin
        r = abs(cross2d(pulley_ps(i,:)-pulley_ps(i-1,:), -pulley_ps(i-1,:)))/norm(pulley_ps(i,:)-pulley_ps(i-1,:));

        f_spring = last_f_spring + k_spring*x_diff;
        torques(i) = f_spring * r;
        
        last_f_spring = f_spring;
        free_len_last = free_len;
        
        if draw
            anchor_ps(i,:) = anchor;
        end
    end
    
    % Do the final anchor calculation just like the initial one
    anchor_thetas(end) = anchor_theta_init + theta_max;
    if anchor_thetas(end)<anchor_thetas(end-1)
       anchor_thetas(end) = anchor_thetas(end-1) + d_theta;
    end
    anchor_final = my_pol2cart([anchor_thetas(end) + theta_max, anchor_r]);
    free_len = norm(anchor_final - pulley_ps(end,:));
    x_diff = free_len - free_len_last;

    %new radius is distance from pull line to origin
    r_final = abs(cross2d(pulley_ps(end,:)-anchor_final, -anchor_final))/norm(pulley_ps(end,:)-anchor_final);

    f_spring_final = last_f_spring + k_spring*x_diff;
    torques(end) = f_spring_final * r_final;
    
    
    if draw
        anchor_ps(end,:) = anchor_final;
        
        figure(100);
        cla reset;
        axis equal
        hold on
        plot(anchor_ps(:,1),anchor_ps(:,2),'b.');
        plot([pulley_ps(:,1); 0; pulley_ps(1,1)],[pulley_ps(:,2); 0; pulley_ps(1,2)],'r.'); 
        plot([0 0 -anchor_offset],[0 -anchor_dist -anchor_dist], 'k', 'LineWidth',2)
        for i = [1 floor(size(pulley_ps,1)/2) size(pulley_ps,1)]
            plot([pulley_ps(i,1) anchor_ps(i,1)],[pulley_ps(i,2) anchor_ps(i,2)],'k--')
        end
        pad = diff(ylim)*0.1;
        ylim(ylim + [-pad pad]);
    end
    
%     input_thetas = anchor_thetas - anchor_theta_init;
%     min_diff = min(diff(input_thetas));
%     if min_diff<0
%         fun = @(angle) -1000*min_diff;
%         %disp(input_thetas);
%         %save input_thetas.mat input_thetas
%         %pause(10);
%     else
        fun = @(angle) interp1(input_thetas,torques,angle);
%     end
        