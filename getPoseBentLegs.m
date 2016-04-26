function x0 = getPoseBentLegs(r)

% A pose with bent knees
% Center of mass aproximately above foot center
x0 = Point(r.getStateFrame);
x0.l_hip_flex = -0.5;
x0.l_knee = 1.5;
x0.l_ankle_flex = -1;
x0.r_hip_flex = -0.5;
x0.r_knee = 1.5;
x0.r_ankle_flex = -1;