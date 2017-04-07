function x0 = getPoseBentLegs(r)

% A pose with bent knees
% Center of mass aproximately above foot center
x0 = Point(r.getStateFrame);
x0.l_leg_hpx = -0.5;
x0.l_leg_kny = 1.5;
x0.l_leg_akx = -1;
x0.r_leg_hpx = -0.5;
x0.r_leg_kny = 1.5;
x0.r_leg_akx = -1;