clear all
close all
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\infoData')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\classdef')


% MY TARGET
x1 = points([3   3],'target');
plotPtArr(x1,'k*')% plot points

global UGVloc
UGVloc = [6  2  pi];
[targets,walls,recFur,cirFur,UGV] = buildAll();
plotAll(targets,walls,recFur,cirFur,UGV); 


% MAIN TESTING PART
% lineX lineY defines line connecting UGV anf the target
lineX = [UGV.loc(1)  x1.loc(1)];
lineY = [UGV.loc(2)  x1.loc(2)];


wall1 = walls(5);
tf = ifLineCross(wall1,lineX,lineY,'show')  % only for reObs

fur1 = recFur(10);


% xs = fur1.vertices(1,:);
% ys = fur1.vertices(2,:);
% % Define and fill a rectangular area in the plane
% 
% mapshow(xs,ys,'DisplayType','polygon','LineStyle','none')
%  
% % Define and display a two-part polyline
% x = [0 10];
% y = [4 0];
% mapshow(x,y,'Marker','+')
% 
% % Intersect the polyline with the rectangle
% [xi, yi] = polyxpoly(x, y, xs, ys);
% 
% if isempty(xi)
%     disp('no such thing');
% else
%     mapshow(xi,yi,'DisplayType','point','Marker','o');
% end




% Functions for setUp
function varargout = buildAll()
% building targets
load('targetLocs');
targetsLoc = locs'; 
targets(1,length(targetsLoc)) = points();  % preallocation with default values
for i = 1:length(targetsLoc)
    targets(1,i)=  points([targetsLoc(i,1)   targetsLoc(i,2)],'target');
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


% build UGV
global UGVloc
fov = [0.3  0.8  1.0472];
size = [0.12  0.1];
UGV = robot(UGVloc,fov,size);


varargout = {targets,walls,recFur,cirFur,UGV};

end


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


function plotAll(targets,walls,recFur,cirFur,UGV)

figure(1)
hold on
xlim([0  9.2])
ylim([0  9.2])
axis equal

% plotPtArr(targets,'k*')% plot all points

for i = 1:10
plotObjArr(walls,'k'); %plotting walls
plotObjArr(recFur,'c'); %plotting obs
end

plotObjArr(cirFur,'c'); %plotting obs

plotRobot(UGV);
% drawFOV(UGV);

end
