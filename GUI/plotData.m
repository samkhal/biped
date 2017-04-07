t = 1:15:45000;
f = figure();
plot(t,response(:,1),'r','DisplayName','Response Angle');
hold on;
plot(t,requested(:,1),'DisplayName','Reference Torque');
title('HipX');
xlabel('milliseconds');
ylabel('radians');
legend('show');
% saveas(f,'Knee Response 6.jpg');

% f = figure();
% plot(t,response(:,2)*2.3,'r','DisplayName','Response');
% hold on;
% plot(t,requested(:,2),'DisplayName','Reference');
% % hold on;
% % plot(t,response(:,2)*2.3-requested(:,2))
% title('Hip X Right');
% xlabel('milliseconds');
% ylabel('radians');
% legend('show');
% saveas(f,'HipXRight.jpg');

% f = figure();
% plot(t,response(:,3),'r','DisplayName','Response');
% hold on;
% plot(t,requested(:,3),'DisplayName','Reference');
% hold on;
% plot(t,response(:,3)*2.3-requested(:,3))
% title('Hip Y Left');
% xlabel('milliseconds');
% ylabel('radians');
% legend('show');
% saveas(f,'HipYLeft.jpg');

% f = figure();
% plot(t,response(:,4),'r','DisplayName','Response');
% hold on;
% plot(t,requested(:,4),'DisplayName','Reference');
% title('Hip Y Right');
% xlabel('milliseconds');
% ylabel('radians');
% legend('show');
% saveas(f,'HipYRight.jpg');
% 
% f = figure();
% plot(t,response(:,5)*1.7,'r','DisplayName','Response');
% hold on;
% plot(t,requested(:,5),'DisplayName','Reference');
% title('Knee Left');
% xlabel('milliseconds');
% ylabel('radians');
% legend('show');
% saveas(f,'KneeLeft.jpg');
% 
% f = figure();
% plot(t,response(:,6),'r','DisplayName','Response');
% hold on;
% plot(t,requested(:,6),'DisplayName','Reference');
% title('Knee Right');
% xlabel('milliseconds');
% ylabel('radians');
% legend('show');
% saveas(f,'KneeRight.jpg');