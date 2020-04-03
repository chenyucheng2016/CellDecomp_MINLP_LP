% Min. 6/25/2017

%{
Update: 
add subclass of recObs and cirObs
add rec furniture group

Todo:
add the rest of furniture group
%}

clear all;
close all;
addpath('C:\Users\Ferrari Lab\Desktop\pathPlanning\classdef')  % add folder of class 


%%%%%%%% NOW TESTING
[targets,walls,obs] = buildAll();
plotAll(targets,walls,obs);   % any chance to modify this?

%%%%%%%% END TESTING


%%%%%%%%% FUNCTIONS

% build env.   varargout = {targets,walls};
function varargout = buildAll()
% building targets
load('targetLocs');
targetsLoc = locs'; 
targets(1,length(targetsLoc)) = points();  % preallocation with default values
for i = 1:length(targetsLoc)
    targets(1,i)=  points(targetsLoc(i,1),targetsLoc(i,2),'target');
end


% build walls
load('walls.mat');
walls(1,length(coors)) = recObs();
for i = 1:length(walls)
    walls(i) = recObs(coors{i},'wall');
end

% build recFur
load('recFurCor.mat');
obs(1,10) = recObs();
for i = 1:length(recFurCor)
    obs(i) = recObs(recFurCor{i},'furniture');
end
varargout = {targets,walls,obs};

end

function plotAll(targets,walls,obs)

figure(1)
hold on
xlim([0  9.2])
ylim([0  9.2])
axis equal

plotPtArr(targets,'k*')% plot all points

plotObjArr(walls,'k'); %plotting walls
plotObjArr(obs,'r'); %plotting walls
alpha(.3)

end

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
        plotRecObs(x,varargin{:});
    end
   
end
