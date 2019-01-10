
% uiopen('C:\Users\kuochen\Desktop\BPTTtest\bikebotAug6\bikebot\bikebot_simulation\sim.txt',1);
sim=textread('sim.txt');
x_collect=sim(1:4,:);
desired_trj_collect=sim(5:6,:);
bem_collect=sim(7,:);
time_collect=sim(8,:);

close all
figure,plot(time_collect,x_collect(1,:))
hold on,plot(time_collect,desired_trj_collect(1,:))
legend('\theta','\theta_d')

figure,plot(time_collect,x_collect(3,:))
hold on,plot(time_collect,desired_trj_collect(2,:))
legend('dot\theta','dot\theta_d')

figure,plot(time_collect,x_collect(2,:))
hold on,plot(time_collect,bem_collect)
legend('\alpha','\alpha_d')
