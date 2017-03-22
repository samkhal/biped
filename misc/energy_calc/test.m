%Ari Goodman
% 3/1/2017
%finds best spring after you load traj and csv as a table!

best_savings = 0;
best_i = 0;
%remove unimportant springs
springsCopy=springs;
toDelete = springs.SuggMaxDeflin>1.75;
springsCopy(toDelete,:) = [];
toDelete = springsCopy.ODin>1;
springsCopy(toDelete,:) = [];
toDelete = springsCopy.SuggMaxLoadlbs< springsCopy.InitialTensionlbs*1.1+springsCopy.Ratelbsin*pi/2*1.1^2; %worst case
springsCopy(toDelete,:) = [];
size(springsCopy)
%for all springs, calculate energy savings
for i=1:height(springsCopy)
constant_range = [.5; 1.1 ];
joints = {'r_leg_kny'};
foo = @(angle, constants)((springsCopy.InitialTensionlbs(i)*constants(1)+constants(1).^2*springsCopy.Ratelbsin(i)*angle)*.1129848);

[const savings] = find_optimal_energy_system(traj, foo, constant_range, joints);
if best_savings(1)<savings(1)
    best_savings=savings;
    best_i=i;
end
end
%146.2145 savings, 34% ...80533