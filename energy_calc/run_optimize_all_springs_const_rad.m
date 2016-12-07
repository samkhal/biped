% Run constant-radius optimization on all of the springs

springs = csvread('springs.csv');
min_r = 0.5; %in
max_r = 3; %in

costs = inf(1,size(springs,1));
for i=1:size(springs,1)
    disp(i)
    f_min = springs(i,4);
    f_max = springs(i,6);
    k = springs(i,3);
    len = springs(i,2);
    deflect_max = springs(i,5);
    
    if len>8 || f_max<10 || deflect_max < min_r*pi/2;
        continue
    end
    
    
    
    xlow =zeros(3,1);xupp =xlow;
    xlow(1)=si(f_min,'lbf');xupp(1)=si(f_max,'lbf'); % f_min
    xlow(2)=si(min_r,'in');xupp(2)=si(max_r,'in'); % r
    xlow(3)=si(k,'lbf/in');xupp(3)=si(k,'lbf/in'); % k
    [pulley_vals, cost] = optimize_pulley_constant_radius(traj,xlow,xupp,si(f_max,'lbf'));
    costs(i) = cost;
end

[best_cost, best_i] = min(costs);