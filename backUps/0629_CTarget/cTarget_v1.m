% Min. 6/27/2017

%{
Update: 
1. modified points class (use loc instead of myX myZ)
2. Robot class, with function to determine current robot location is in
C-target (inCTarget) an can plot FOV (drawFOV)


Todo:C-target
1. regardless of C-obstacle, simulate C-target
2. add in-obstacles for recObs and cirObs
3. for every point not in obstacle(C-free) check if in FOV

Problem:
cannot combine cirFur and recFur
can't adjust color  by [1,0,1]
can't change alpha on cirObs

%}

% clear all;
 close all;
addpath('C:\Users\Ferrari Lab\Desktop\pathPlanning\classdef')  % add folder of class 
addpath('C:\Users\Ferrari Lab\Desktop\pathPlanning\infoData')

%%%%%%%% MAIN
[targets,walls,recFur,cirFur] = buildAll();


% NOW TESTING
% A = CTarget(targets);
plotCTarget(A);
hold on
plotAll(targets,walls,recFur,cirFur);   % any chance to modify this?
% END TESTING


%%%%%%%%% FUNCTIONS

% % build env.   varargout = {targets,walls};
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



varargout = {targets,walls,recFur,cirFur};

end

% generate a C-target distribution in the worldSpace
function A = CTarget(targets)
n = 100;  % # of step of sampling
myStart = 0;
myEnd = 9.2;
step = (myEnd-myStart)/n;
xs = linspace(myStart,myEnd,n);
ys = linspace(myStart,myEnd,n);
A = zeros(n,n);

% build a test robot
fov = [0.3  0.8  1.0472];
loc = [0   0   pi];
size = [0.1  0.12];
tester = robot(loc,fov,size);
for i = 1:length(xs)
    for j = 1:length(ys)
        tester.loc = [myStart+(i-1)*step+ step/2  myStart+(j-1)*step+step/2  tester.loc(3)] ;
        if inCTarget(tester,targets)
            A(i,j) = 1;
        end
    end
end
end


% % plot functions

% plot all elements, such as targets, obstacles
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

function plotCTarget(A)
x = [0 9.2];
y = [0 9.2];
% colormap(gray)
% im.AlphaData = .2;
imagesc(x,y,A)

cmap = [.5 .5 .5 %// light gray
        1  1  1] %// white
colormap(cmap)
% colorbar('Ytick',[.25 .75],'Yticklabel',[0 1]) 
% colorbar
set(gca,'YDir','normal')  % otherwise the y axis is flipped
end
