
figure(11)
cla;
hold on;
for i = 2:length(consts)
%for i = 13
    x = linspace(.67,pi/2,100);
    plot(x,polyval(consts{i-1},x+consts{1}));
    pause(1)
end