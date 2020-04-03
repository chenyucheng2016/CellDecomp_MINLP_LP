% Min. 6/23/2017

%{
Update: 
modify plot obastacle function with varargin; put plot single object in
class methods
moified to 30 targets as human study

Todo:
group the buildAll function and plotAll function
%}

clear all;
close all;
addpath('C:\Users\Ferrari Lab\Desktop\pathPlanning\classdef')  % add folder of class 


% building targets
load('targetLocs');
targetsLoc = locs'; 
targets(1,length(targetsLoc)) = points();  % preallocation with default values
for i = 1:length(targetsLoc)
    targets(1,i)=  points(targetsLoc(i,1),targetsLoc(i,2),'target');
end

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

plotPtArr(targets,'k*')% plot all points

plotObjArr(walls,'k'); %plotting walls
alpha(.5)
%%%%%%%% END TESTING

% plot functions
% plot for points
function plotPtArr(ptArr,varargin)

% if nargin == 1
    for x = ptArr
        plotPt(x,varargin{:})
    end
% end
end

%  draws array of obstacle objects, color (e.g.'k') and hollow(true) is optional.
function plotObjArr(obsArr,varargin)
    for x = obsArr
        plotObs(x,varargin{:});
    end
   
end
