close all
clear all
addpath('C:\Users\Ferrari Lab\Desktop\pathPlanning\classdef')  % add folder of class 


%% build robot
fov = [0.6  1  1.0472];

% weBotLoc = [4.49065   3.88421];
% loc = [weBotLoc(2)+0.1  weBotLoc(1)+0.7]
loc = [8    5.1906   pi];
% loc = [3.9842    5.1906   pi];  % in matlab, including direction (TEST CASE1, as opened in webot)
% loc = [2.8    5.1906   0];  % (TEST CASE2)
% loc = [3.5    4.6   pi/2]; %(TEST CASE3) 
size = [0.1  0.12];
myRobot = robot(loc,fov,size);



%% build target
tar1 =  points([2   2],'target');
tar2 =  points([3.8   4.8],'target');
tar3 =  points([3.2   5],'target');
tar4 =  points([11   8],'target');
tar5 =  points([10   5],'target');
tar6 =  points([6   3],'target');

tar7 =  points([3.6   5.2],'target');
tar8 =  points([3.6    5.1],'target');
tar9 =  points([2.8   5.6],'target');
tar10 =  points([3.2   5.4],'target');
targets = [tar1  tar2  tar3  tar4   tar5   tar6  tar7  tar8   tar9   tar10];
%% plot
figure(1)
hold on
xlim([0  9.2])
ylim([0  9.2])
axis equal

plotRobot(myRobot);


% % test if a pt is in FOV
% for i = 1:length(targets)
% 
% flag = ptInFOV(myRobot, targets(i));
% 
% if flag
%     plotPt(targets(i),'r*');
% else
%     plotPt(targets(i),'k*');
% 
% end
% % fprintf('%d\n',round(a));
% end

for i = 1:length(targets)

flag = ptInFOV(myRobot, targets(i));

if flag
    plotPt(targets(i),'r*');
else
    plotPt(targets(i),'k*');

end
% fprintf('%d\n',round(a));
end


drawFOV(myRobot);
inCTarget = inCTarget(myRobot,targets)
%% other testings

%% test anglePtRobot
% plotRobot(myRobot);
% plotPt(tar2,'k*');
% disp(radtodeg(anglePtRobot(myRobot,tar2)))

