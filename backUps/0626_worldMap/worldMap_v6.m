% Min. 6/26/2017

%{
Update: 
add circle furniture group
modify cirObs subclass's plot

Todo:
C-obstacle

Problem:
cannot combine cirFur and recFur
can't fully adjust color  by [1,0,1]
can't change alpha on cirObs

%}

clear all;
close all;
addpath('C:\Users\Ferrari Lab\Desktop\pathPlanning\classdef')  % add folder of class 


%%%%%%%% NOW TESTING
[targets,walls,recFur,cirFur] = buildAll();
plotAll(targets,walls,recFur,cirFur);   % any chance to modify this?

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
recFur(1,10) = recObs();
for i = 1:10
    recFur(i) = recObs(recFurCor{i},'furniture');
end

% build cirFur
load('cirFurCor.mat');
cirFur(1,4) = cirObs();
for i = 1:4
    cirFur(i) = cirObs(cirFurCor{i}(1:2),cirFurCor{i}(3),'furniture');
end



varargout = {targets,walls,recFur,cirFur};

end

function plotAll(targets,walls,recFur,cirFur)

figure(1)
hold on
xlim([0  9.2])
ylim([0  9.2])
axis equal

plotPtArr(targets,'k*')% plot all points

plotObjArr(walls,'k'); %plotting walls
plotObjArr(recFur,'c'); %plotting obs
plotObjArr(cirFur,'c'); %plotting obs
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
        plotObs(x,varargin{:});
    end
   
end
