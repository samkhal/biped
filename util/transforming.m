%rpy xyz
T = @(x) [rpy2rotmat(x(1:3)) x(4:6)';0 0 0 1];
rpyxyz = @(x) [rotmat2rpy(x(1:3,1:3))' x(1:3,4)'];

%---------left leg
p2h_atlas = T([0 0 0 0 0.089 0]);
p2h_cam = T([1.56567  0      -1.57079 0.0107    0.05539   -0.10919]);

h2u_atlas = T([0 0 0 0.05 0.0225 -0.066]);
h2u_cam = T([0  0  0 -8.05900000e-02  -1.00000000e-05   3.02000000e-03]);

h_cam2u_atlas = inv(p2h_cam)*p2h_atlas*h2u_atlas;
%rpyxyz(h_cam2u_atlas)

u2l_atlas = T([0 0 0 -0.05 0 -0.374]);
l2a_atlas = T([0 0 0 0 0 -0.422]);
a2f_atlas = T([0 0 0 0 0 0]);

u2l_cam = T([0  0  0 0.00373 -0.20954 -0.00107]);
l2a_cam = T([0  0  0 -1.50000000e-04  -2.21290000e-01  -1.13000000e-03]);
a2f_cam = T([-1.5708  1.5708 0 -0.00077 -0.0179  -0.06914]);
f2fv_cam = T([1.5708  0 -1.5708 0.00112  -0.07394 0.14666]);

a_cam2f_atlas = inv(l2a_cam)*inv(u2l_cam)*inv(h2u_cam)*inv(p2h_cam)*p2h_atlas*h2u_atlas*u2l_atlas*l2a_atlas*a2f_atlas;
a_cam2f_atlas_rpy = rpyxyz(a_cam2f_atlas);
a_cam2f_atlas_rpy_new = a_cam2f_atlas_rpy;
a_cam2f_atlas_rpy_new(4) = 0;
a_cam2f_atlas_rpy_new(5) = a_cam2f_atlas_rpy_new(5) + 0.3;
a_cam2f_atlas_new = T(a_cam2f_atlas_rpy_new);

f_atlas2fv_cam = inv(a_cam2f_atlas_new)*a2f_cam*f2fv_cam;
%rpyxyz(f_atlas2fv_cam)



%----------------right leg
p2h_atlas = T([0 0 0 0 -0.089 0]);
p2h_cam = T([1.56567  0      -1.57079 0.00455  -0.05573  -0.10919]);

h2u_atlas = T([0 0 0 0.05 -0.0225 -0.066]);
h2u_cam = T([0  0  0 7.52400000e-02   1.00000000e-05  -3.13000000e-03]);

h_cam2u_atlas = inv(p2h_cam)*p2h_atlas*h2u_atlas;
%rpyxyz(h_cam2u_atlas)

u2l_atlas = T([0 0 0 -0.05 0 -0.374]);
l2a_atlas = T([0 0 0 0 0 -0.422]);
a2f_atlas = T([0 0 0 0 0 0]);

u2l_cam = T([0  0  0 0.00328 -0.20954 -0.00107]);
l2a_cam = T([0  0  0 4.00000000e-05  -2.21290000e-01  -1.13000000e-03]);
a2f_cam = T([-1.5708  1.5708 0 -0.00079 -0.01789 -0.07124]);
f2fv_cam = T([1.5708  0 -1.5708 -0.00097  0.19275  0.14664]);

a_cam2f_atlas = inv(l2a_cam)*inv(u2l_cam)*inv(h2u_cam)*inv(p2h_cam)*p2h_atlas*h2u_atlas*u2l_atlas*l2a_atlas*a2f_atlas;
a_cam2f_atlas_rpy = rpyxyz(a_cam2f_atlas);
a_cam2f_atlas_rpy_new = a_cam2f_atlas_rpy;
a_cam2f_atlas_rpy_new(4) = 0;
a_cam2f_atlas_rpy_new(5) = a_cam2f_atlas_rpy_new(5) + 0.3;
a_cam2f_atlas_new = T(a_cam2f_atlas_rpy_new);

f_atlas2fv_cam = inv(a_cam2f_atlas_new)*a2f_cam*f2fv_cam;
%rpyxyz(f_atlas2fv_cam)

heel_cam = [-0.1015 -0.0139 -0.048 1]';
heel_atlas = inv(a_cam2f_atlas_new)*a2f_cam*heel_cam;
heel_atlas'

