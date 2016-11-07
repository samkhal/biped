%Ari Goodman
%11/5/2016

%finds the torque for a given angle, with a preset cam equation

%% inputs
%angle is the angle of the knee joint in rad
%torque is Nm

%% outputs torque

%TODO: RESUSE THIS FUNCTION AND REMOVE A BUNCH OF MATLAB FILES BY HAVING
%THIS FUNCTION TAKE A FUNCTION.
function torque = angle2torqueCams(angle)

if(angle< pi/2) disp('error: below minimum') %for logan
elseif(angle>pi) disp('error: above maximum')
end

L_U = .203;% length of top leg m
L_L = .2213;% length of bottom leg m
M_body = 3.245;% mass of body/2 kg
M_U = .4; %mass of upper leg kg
g = 9.81; %m/s^2 grav constant

%torque =
%1000*L_U*(L_L*sin(angle))/((L_U^2+L_L^2-2*L_U*L_L*cos(angle))^.5)*(M_body+M_U/2)*g;
%%logans old equation
torque = -5.8548e+04+(9.3233e+04).*angle+(-3.3525e+04)*abs(angle.^2); %hard coded best 2nd order equation found so far
end