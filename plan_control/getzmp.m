function [xzmp,yzmp] = getzmp()
%getxmp gets the current x and y total system zmp
options.floating=true;
legs=RigidBodyManipulator('urdf2/Legs.urdf',options);
nq=legs.getNumPositions();

pelvis = legs.findLinkId('pelvis');
l_hip = legs.findLinkId('Biped__Left_Hip');
r_hip = legs.findLinkId('Biped__Right_Hip');
l_top_leg = legs.findLinkId('Biped__Top_Left_Leg');
r_top_leg = legs.findLinkId('Biped__Top_Right_Leg');
l_bot_leg = legs.findLinkId('Biped__Bottom_Left_Leg');
r_bot_leg = legs.findLinkId('Biped__Bottom_Right_Leg');
l_ankle = legs.findLinkId('Biped__Left_Ankle');
r_ankle = legs.findLinkId('Biped__Right_Ankle');
l_foot = legs.findLinkId('l_foot');
r_foot = legs.findLinkId('r_foot');
l_toe = legs.findLinkId('l_toe');
r_toe = legs.findLinkId('r_toe');

g=9.81;
%Vector order [pelvis, left hip, top left leg, bottom left leg, left ankle


m(1)=legs.body(pelvis).mass;
m(2)=legs.body(l_hip).mass;
m(3)=legs.body(l_top_leg).mass;
m(4)=legs.body(l_bot_leg).mass;
m(5)=legs.body(l_ankle).mass;
m(6)=legs.body(l_foot).mass;
m(7)=legs.body(l_toe).mass;
m(8)=legs.body(r_hip).mass;
m(9)=legs.body(r_top_leg).mass;
m(10)=legs.body(r_bot_leg).mass;
m(11)=legs.body(r_ankle).mass;
m(12)=legs.body(r_foot).mass;
m(13)=legs.body(r_toe).mass;

I1=legs.body(pelvis).inertia;
I2=legs.body(l_hip).inertia;
I3=legs.body(l_top_leg).inertia;
I4=legs.body(l_bot_leg).inertia;
I5=legs.body(l_ankle).inertia;
I6=legs.body(l_foot).inertia;
I7=legs.body(l_toe).inertia;
I8=legs.body(r_hip).inertia;
I9=legs.body(r_top_leg).inertia;
I10=legs.body(r_bot_leg).inertia;
I11=legs.body(r_ankle).inertia;
I12=legs.body(r_foot).inertia;
I13=legs.body(r_toe).inertia;

Ix=[I1(1) I2(1) I3(1) I4(1) I5(1) I6(1) I7(1) I8(1) I9(1) I10(1) I11(1) I12(1) I13(1)];
Iy=[I1(5) I2(5) I3(5) I4(5) I5(5) I6(5) I7(5) I8(5) I9(5) I10(5) I11(5) I12(5) I13(5)];

com1=legs.body(pelvis).com;
com2=legs.body(l_hip).com;
com3=legs.body(l_top_leg).com;
com4=legs.body(l_bot_leg).com;
com5=legs.body(l_ankle).com;
com6=legs.body(l_foot).com;
com7=legs.body(l_toe).com;
com8=legs.body(r_hip).com;
com9=legs.body(r_top_leg).com;
com10=legs.body(r_bot_leg).com;
com11=legs.body(r_ankle).com;
com12=legs.body(r_foot).com;
com13=legs.body(r_toe).com;

x=[com1(1) com2(1) com3(1) com4(1) com5(1) com6(1) com7(1) com8(1) com9(1) com10(1) com11(1) com12(1) com13(1)];
y=[com1(2) com2(2) com3(2) com4(2) com5(2) com6(2) com7(2) com8(2) com9(2) com10(2) com11(2) com12(2) com13(2)];
z=[com1(3) com2(3) com3(3) com4(3) com5(3) com6(3) com7(3) com8(3) com9(3) com10(3) com11(3) com12(3) com13(3)];

xddot=;
zddot=;

thetaxddot=;
thetayddot=;


xzmp=0;
yzmp=0;
for i=1:13
    
    xzmp=xzmp+((m(i)*(zddot(i)+g)*x(i))-(m(i)*xddot(i)*z(i))-(Iy(i)*thetayddot(i)))/(m(i)*(zddot(i)+g));
    yzmp=yzmp+((m(i)*(zddot(i)+g)*y(i))-(m(i)*xddot(i)*z(i))-(Ix(i)*thetaxddot(i)))/(m(i)*(zddot(i)+g));
end

end

