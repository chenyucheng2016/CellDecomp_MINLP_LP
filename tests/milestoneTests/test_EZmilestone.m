% easy version milestone test
clear all; 
% build a narrow passage using two obstacles


% close all;
% addpath('/Users/min/Google Drive/pathPlanning(1)/infoData')
% addpath('/Users/min/Google Drive/pathPlanning(1)/classdef')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\infoData')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\classdef')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\helperFuns')
warning off

%%%%%%%% MAIN
[targets,walls,recFur,cirFur,UGV] = buildAll();


global mapLimit A3d A2d  % setting for canvas size 
 mapLimit = [0  5];
%  figure;
%  plotAll(all); 
%  hold on 
 
 % % C-free
 obstacles = {walls,recFur,cirFur};
A3d = CFree(UGV,obstacles);
% A3d = load('test_A3d_400x40'); A3d = A3d.A3d;
% A3d = load('test_A3d_120x2'); A3d = A3d.A3d;
% A3d = load('test_A3d_15x1'); A3d = A3d.A3d;
 A2d = A3d(:,:,1); % 2d projection
ms = genMS();
% ms = [];
all = {targets,walls,recFur,cirFur,UGV,ms};
% % 2D plot C target
figure;
plotCTarget(A2d); % 2d C free
hold all
 plotAll(all); 


%% NOW TESTING

% setting
function ms = genMS()
global A2d
n = length(A2d);
numMS = 80;  % number of total milestone
w = 0.5;
nUnif = round(numMS * w);
nRBB = numMS - nUnif;
sigma = round(n*0.08);
msRBB = [];   msUnif = [];
maxLoop = 20; % need to abandon a point if dead loop
step = 5;  % r for orthogonal test for RBB
while length(msRBB) < nRBB || length(msUnif) < nUnif
    x1 =  randi(n);
    y1 =  randi(n);
    
    % in obstacle?
    if A2d(x1,y1) == 1 %  in C-free
        if length(msUnif) < nUnif
            [x,y] = grid2xy(x1,y1);
            msUnif = [msUnif  milestone([x  y],'uniform')];
        end
    else  % in CB
        success = false; % true if find x2,y2 ro form a ms
        count = 0;
        while success ~= true && length(msRBB) < nRBB && count < maxLoop
            x2 = round(normrnd(x1,sigma));
            y2 = round(normrnd(y1,sigma));
            if x2>0 && y2>0 && x2<n && y2<n && A2d(x2,y2) == 0 %  x2,y2 in CB
                xq = round((x1 + x2)/2);
                yq = round((y1 + y2)/2);
                if  A2d(xq,yq) == 1 && orthoTest([x1  y1],[x2   y2],step)  % q in C free
                    [x_rbb,y_rbb] = grid2xy(xq,yq);
                    msRBB = [msRBB  milestone([x_rbb,y_rbb],'RBB')];
                    success = true;
                end
            end
            count = count+1;
        end
        
    end
    
end
ms = [msUnif  msRBB];
end

function [x,y] = grid2xy(i,j)
global mapLimit B3d A2d
 n = length(A2d);% m = length(B3d{1});
myStart = mapLimit(1); myEnd = mapLimit(2);
step = (myEnd-myStart)/(n-1);
x = myStart+(j-1)*step;
y = myStart+(i-1)*step;

end


function flag = orthoTest(p1,p2,step)
global A2d
n = length(A2d);
flag = true;
x1 = p1(1); y1 = p1(2);
x2 = p2(1); y2 = p2(2);

xq = (x1 + x2)/2;   yq = (y1 + y2)/2; 
ang = atan2(x2-x1,y1-y2);
for r = 1:step
    x3 = round(xq+cos(ang)*r);
    y3 = round(yq+sin(ang)*r);
    if  x3>0 && y3>0 && x3<n && y3<n
        if A2d(x3,y3) == 0  % endpoint in CB
            flag = false; 
            return
        end
    end
    x4 = round(xq-cos(ang)*r);
    y4 = round(yq-sin(ang)*r);
   
    if  x4>0 && y4>0 && x4<n && y4<n
        if A2d(x4,y4) == 0
            flag = false;
            return
        end
    end
end
end




% % build env.   varargout = {targets,walls};
function varargout = buildAll()
% building targets
load('targetLocs');
targetsLoc = locs'; 

for i = 9
    targets(1,1)=  points([targetsLoc(i,1)   targetsLoc(i,2)],'target');
end


% build walls
Wallcor = load('walls.mat'); Wallcor = Wallcor.coors;
 walls = recObs(Wallcor{5},'wall');

% build recFur
load('recFurCor.mat');
% recFur = recObs([2  2  4.3   4.3;1.5  2.5  2.5  1.5],'furniture');
recFur = recObs([2  2  4   4;1.5  2.6  2.6  1.5],'furniture');

% % build cirFur
% load('cirFurCor.mat');
% i = 2;
% % cirFur = cirObs(cirFurCor{i}(1:2),cirFurCor{i}(3),'furniture');
cirFur = [];

% build UGV
fov = [0.3  0.8  pi/4];
loc = [4.5906  4.5842  pi];
size = [0.12  0.1];
UGV = robot(loc,fov,size);


varargout = {targets,walls,recFur,cirFur,UGV};

end
 

% generate a C-target distribution in the worldSpace
% generate a C-target distribution in the worldSpace
function A = CFree(robot,obstacles)
% build a tester circle zoom
w = robot.size(1); h = robot.size(2);
r = sqrt(w^2+h^2)/2;

% plotObs(robotCir,'r',true);

global mapLimit
n = 100;  % # of step of sampling
m = 1; % theta steps

myStart = mapLimit(1);
myEnd = mapLimit(2);

step = (myEnd-myStart)/n;
xs = linspace(myStart,myEnd,n);
ys = linspace(myStart,myEnd,n);
A = zeros(n,n,m);
tempA = zeros(n,n);
% ts = linspace(0,2*pi,m+1); % theta 

for i = 1:length(xs)
    for j = 1:length(ys)
        robotCir = cirObs([myStart+(j-1)*step+step/2  myStart+(i-1)*step+step/2],r,'tester');
        if inCFree(robot,robotCir,obstacles)
            % A(i,j,k) = 1;
            tempA(i,j) = 1;      
        end
    end
% end

% make 2D C-free into  3D

for p = 1:m
    A(:,:,p) = tempA;
end
end
end

% % plot functions
function plotCTarget(B)
global mapLimit
x = mapLimit;y = mapLimit;
% x = [0 9.2];
% y = [0 9.2];
% colormap(gray)
% im.AlphaData = .2;
 imagesc(x,y,B)

cmap = [.6 .6 .6 %// light gray
        1  1  1];%// white
colormap(cmap)
% colorbar('Ytick',[.25 .75],'Yticklabel',[0 1]) 
% colorbar
set(gca,'YDir','normal')  % otherwise the y axis is flipped
end

% plot all elements, such as targets, obstacles
function plotAll(all)
% all = {targets,walls,recFur,cirFur,UGV};
targets = all{1}; walls =  all{2}; recFur = all{3}; cirFur = all{4}; UGV = all{5};  ms = all{6};
hold on
global mapLimit
% xlim([0  9.2])
% ylim([0  9.2])
xlim([mapLimit(1)  mapLimit(2)])
ylim([mapLimit(1)  mapLimit(2)])
axis equal

plotPtArr(targets,'r*')% plot all points
for myMS = ms
    plotPtArr(myMS,'g*')% plot all milestones
end

for i = 1:10
plotObjArr(walls,'k'); %plotting walls
plotObjArr(recFur,'k'); %plotting obs
end

plotObjArr(cirFur,'c'); %plotting obs

% plotRobot(UGV);
% drawFOV(UGV);

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
% 2D

