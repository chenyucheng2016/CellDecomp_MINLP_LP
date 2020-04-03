% Min. 6/22/2017

%{
Update:
construct walls, plot functions for obstacle
points, obstacles class

Todo:
plot functions for points
%}

clear all;
close all;
addpath('C:\Users\Ferrari Lab\Desktop\pathPlanning\classdef')  % add folder of class 

% testings
% a = points;
% a.myX = 3;
% a.myZ = 5;
% a.type = 'target';

% initialize by for loop and array
% arr = [];
% for i =1:7
%     arr = [arr, points(i,i,'target')];
% end

% building targets
%targetsLoc = [(5, 5.7), (5, 7.5), (8.9, 2), (7, 7), (4.5, 1.3), (9.6, 2.7), (11, 5.2), (6, 5.2), (6, 6.5), (1, 5.2)]
% targetsLoc = [5   5.7; 5  7.5;  8.9	 2;	7	7; 4.5  1.3; 9.6  2.7; 11  5.2; 6 5.2; 6  6.5; 1  5.2];
% 
% targets(1,length(targetsLoc)) = points();
% for i = 1:length(targetsLoc)
%     targets(1,i)=  points(targetsLoc(i,1),targetsLoc(i,2),'target');
% end

% build walls
load('walls.mat');
walls(1,length(coors)) = obstacles();
for i = 1:length(walls)
    x = coors{i}(1,:);
    y = coors{i}(2,:);
    walls(i) = obstacles(coors{i},'wall');
end


%%%%%%%% NOW TESTING
figure(1)
hold on
xlim([0  9.2])
ylim([0  9.2])
plotObjArr(walls,'k')
alpha(.5)
%%%%%%%% END TESTING


% plot functions

% draws a single obstacle object, color (e.g.'k') and hollow(true) is optional.
function plotSingleObs(obs,edgeColor, hollow)
if nargin == 1
    plot(obs.vertices(1,:), obs.vertices(2,:));
    fill(obs.vertices(1,:), obs.vertices(2,:));
elseif nargin == 2
    plot(obs.vertices(1,:), obs.vertices(2,:),edgeColor);
    fill(obs.vertices(1,:), obs.vertices(2,:),edgeColor);
elseif  nargin == 3
    if hollow == true
        polyEdge(obs.vertices(1,:), obs.vertices(2,:),edgeColor);
    end
end
end
% draws array of obstacle objects, color (e.g.'k') and hollow(true) is optional.
function plotObjArr(obsArr,edgeColor,hollow)

if nargin == 1
    for x = obsArr
        plotSingleObs(x,'k')
    end
elseif nargin == 2
    for x = obsArr
        plotSingleObs(x,edgeColor)
    end
else
    for x = obsArr
        plotSingleObs(x,edgeColor,hollow)
    end
end

end

% draws a hollow countour according to coordinate arrays xs, ys. color is
% optinonal, default is random
function polyEdge(xs,ys,color) 

if nargin == 2
    plot([xs(1,:)  xs(1)], [ys(1,:)  ys(1)]);
elseif nargin == 3
    plot([xs(1,:)  xs(1)], [ys(1,:)  ys(1)],color);
end
    
end