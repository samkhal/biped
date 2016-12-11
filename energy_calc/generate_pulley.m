%Sam Khalandovsky
% Variable radius pulley design based on torque profile
% All units in SI
% Points are represented as [x y] pairs

% Inputs:
% tau_profile, function handle to angle->torque mapping. Should be
% decreasing.
% anchor_dist, distance along link from anchor to pivot (y axis)
% anchor_offset, offset of anchor from link (-x axis)
% theta_max, maximum angle of deflection 
% theta_res, resolution of pulley approximation 
% spring_max, maximum force of the spring
% k_spring, spring constant 
% draw, create pulley plot (default: false)

% Outputs:
% pulley_points, an nx2 matrix of polygon points defining the pulley

function pulley_ps = generate_pulley(tau_profile, anchor_dist, anchor_offset, theta_max, theta_res, spring_max, k_spring, draw)

assert(anchor_offset>=0, 'anchor_offset must be positive (measured in -x direction)');
assert(anchor_dist>0, 'anchor_distance must be positive (measured in -y direction)');
assert(theta_max<2*pi, 'theta_max must be less than 2 pi (in radians)');

if nargin<8
    draw = false;
end

% define virtual anchor points as pulley is unwrapped
anchor_r = hypot(anchor_dist, anchor_offset);
anchor_theta = atan(anchor_offset/anchor_dist);
thetas = (0:theta_res:theta_max);
anchor_ps(:,1) = -anchor_r*sin(thetas+anchor_theta);
anchor_ps(:,2) = -anchor_r*cos(thetas+anchor_theta);

% initalize the pulley polgon points
pulley_ps = zeros(length(thetas),2);

r0 = tau_profile(thetas(1))/spring_max; % radius at maximum force
% This initial point will be dropped at the end
assert(0<r0 && r0<anchor_r, 'anchor must be farther from the joint than the initial minimum radius')
pulley_ps(1,1) = -r0*sin(anchor_theta+acos(r0/anchor_r)); %x
pulley_ps(1,2) = -r0*cos(anchor_theta+acos(r0/anchor_r)); %y

spring_forces = zeros(length(thetas),1);
spring_forces(1) = spring_max;

% Function to calculate next torque
% i, index of torque to calculate
% alpha_dist, distance in +X along previous pull line to new alpha point
function [torque, alpha, f_spring, r] = next_torque(i, alpha_dist)

    last_anchor_to_pulley = pulley_ps(i-1,:)-anchor_ps(i-1,:);
    alpha = pulley_ps(i-1,:) + last_anchor_to_pulley/norm(last_anchor_to_pulley) * alpha_dist;
    free_len = norm(anchor_ps(i,:) - alpha);
    free_len_last = alpha_dist + norm(last_anchor_to_pulley);
    x_diff = free_len - free_len_last;
    
    %new radius is distance from pull line to origin
    cross2d = @(a,b) subsref(cross(horzcat(a,0),horzcat(b,0)),substruct('()',{3}));
    r = abs(cross2d(alpha-anchor_ps(i,:),-anchor_ps(i,:)))/norm(alpha-anchor_ps(i,:));
        
    f_spring = spring_forces(i-1) + k_spring*x_diff;
    torque = f_spring * r;
end

tol = 0.001; %tolerance for exactness in torque
max_error = 0;
for i = 2:length(thetas)
    optim_fun = @(alpha_dist) abs(tau_profile(thetas(i)) - next_torque(i, alpha_dist));
    %alpha_dist = fminsearch(optim_fun, 0);
    [alpha_dist, fval] = fminbnd(optim_fun, 0, anchor_dist); %anchor_dist is just some reasonable upper bound
    max_error = max(max_error,fval);
        
    [~, alpha, spring_forces(i), ~] = next_torque(i, alpha_dist);
    pulley_ps(i,:) = alpha;
end

if max_error>tol
    warning(['max_error too high: ',num2str(max_error)]);
end



%discard first point by setting it equal to second
pulley_ps(1,:) = pulley_ps(2,:);

if draw
    figure(99);
    cla reset;
    axis equal
    hold on
    plot(anchor_ps(:,1),anchor_ps(:,2),'b');
    plot([pulley_ps(:,1); 0; pulley_ps(1,1)],[pulley_ps(:,2); 0; pulley_ps(1,2)],'r.'); 
    plot([0 0 -anchor_offset],[0 -anchor_dist -anchor_dist], 'k', 'LineWidth',2)
    for i = [1 floor(length(thetas)/2) length(thetas)]
        plot([pulley_ps(i,1) anchor_ps(i,1)],[pulley_ps(i,2) anchor_ps(i,2)],'k--')
    end
    pad = diff(ylim)*0.1;
    ylim(ylim + [-pad pad]);
end
end