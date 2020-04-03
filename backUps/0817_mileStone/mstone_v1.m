% Min. 8/8/2017
% Stage Goal: Load Cfree, Ctarget from mat. Compute PRM(narrow passage,
% EER) and generate milestones.

%{
Update: 
1.add class target<points, with target properties(curLev etc.)  
& modified class points, delete 'type' (can create another subclass for milestone)

ToDos:

Notes:

%}

clear all; 
% close all;
% addpath('/Users/min/Google Drive/pathPlanning(1)/infoData')
% addpath('/Users/min/Google Drive/pathPlanning(1)/classdef')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\infoData')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\classdef')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\helperFuns')
warning off

%%%%%%%% MAIN
[targets,walls,recFur,cirFur,UGV] = buildAll();
all = {targets,walls,recFur,cirFur,UGV};

global mapLimit   % setting for canvas size 
 mapLimit = [0  9.2];
%  figure;
%  plotAll(all); 
 hold on 

% NOW TESTING
obstacles = {walls,recFur,cirFur};

global  A2d A3d B3d

load ('A3d_150x40');
A2d = A3d(:,:,1); % 2d projection
B3d = load('B3d_150x40');  B3d = B3d.B3d;

% PRM(targets)
% EER = EERcalc();

% END TESTING

%%%%%%%%% FUNCTIONS

% generate milestone, main function
function points = PRM(targets)
global A2d mapLimit B3d
Asize = size(A2d); n = Asize(1);% # of step of sampling
m = length(B3d);

% initilize variables
% I: not in C free    U: in C free  V: EER
I = zeros(n,n,m); U = I; V = I;

for k = 1:m
for i = 1:n
    for j = 1:n
        % j indicates increment in x
        % loc = [myStart+(j-1)*step+step/2  myStart+(i-1)*step+step/2  currentTheta] ;
        if A2d(i,j) == 1 %  in C-free
            U(i,j,k) = 1; I(i,j,k) = 0; 
            V(i,j,k)=Vcalc(targets,i,j,k);   %$ function EER, determine current EER
        else
            U(i,j,k) = 0; I(i,j,k) = 1; V(i,j,k)=0;
        end
    end
end
end
U = U/sum(U(:));  V = V/sum(V(:));  
points = [];
end

% (EER (V) functions
% calculate initial EER
function V = Vcalc(targets,i,j,k)
B3d = load('B3d_150x40'); B3d = B3d.B3d;
if B3d{k}(i,j) == 1 % in Ctarget
    [x,y] = grid2xy(i,j);
    nearestID = nearestTar(targets,x,y);
    V = EERcalc(targets(nearestID));
else
    V = 0;
end

end

function [x,y] = grid2xy(i,j)
global mapLimit B3d
n = length(B3d); m = length(B3d{1});
myStart = mapLimit(1); myEnd = mapLimit(2);
step = (myEnd-myStart)/(n-1);
x = myStart+(j-1)*step;
y = myStart+(i-1)*step;

end
%  precondition, pos in C-Target2D
function nearestID = nearestTar(targets,x,y) 
nearestID = 0;
targetID = 0; dist = inf;
    for target = targets
        targetID = targetID+1;
        thisDist = distPts(target.loc(1),target.loc(2),x,y) ;
        if thisDist < dist
            nearestID = targetID;
            dist = thisDist;
        end
    end
end

% given the target (current cueLevel and final id) get oneStep EER
% final id is just for calculating id at current level.
function EER = EERcalc(target) % 
oneStep = load('EER_oneStep');  oneStep = oneStep.oneStep;
crt_cueLev = target.crt_cueLev; id = target.id;
if crt_cueLev == 3 % no more cues to reveal 
    EER = 0;
    return
else
    id_crt =  ceil(id/2^(3-crt_cueLev));
    EER = oneStep{crt_cueLev+1}(id_crt);
end
end
% )EER (V) functions


% % build env.   varargout = {targets,walls};
function varargout = buildAll()
% building targets
load('targetLocs');
targetsLoc = locs'; 
targetInfo = load('targetInfo'); targetInfo = targetInfo.targetInfo; targetInfo = targetInfo';
targets(1,length(targetsLoc)) = target();  % preallocation with default values
for i = 1:length(targetsLoc)
    % loc,type,                                                      init_cueLev,id,y_ans
    targets(1,i)=  target([targetsLoc(i,1)   targetsLoc(i,2)],targetInfo(i,3),targetInfo(i,1),targetInfo(i,2));
end


% build walls
Wallcor = load('walls.mat'); Wallcor = Wallcor.coors;
walls(1,length(Wallcor)) = recObs();
for i = 1:length(walls)
    walls(i) = recObs(Wallcor{i},'wall');
end

% build recFur
recFurCor = load('recFurCor.mat'); recFurCor = recFurCor.recFurCor;
recFur(1,length(recFurCor)) = recObs();
for i = 1:length(recFurCor)
    recFur(i) = recObs(recFurCor{i},'furniture');
end

% build cirFur
cirFurCor = load('cirFurCor.mat'); cirFurCor = cirFurCor.cirFurCor;
cirFur(1,length(cirFurCor)) = cirObs(); 
for i = 1:length(cirFurCor)
    cirFur(i) = cirObs(cirFurCor{i}(1:2),cirFurCor{i}(3),'furniture');
end


% build UGV
fov = [0.3  0.8  1.0472];
loc = [4.5906  4.5842  pi];
size = [0.12  0.1];
UGV = robot(loc,fov,size);


varargout = {targets,walls,recFur,cirFur,UGV};

end

% 2D Ctarget
function plotCTarget(B)
global mapLimit
x = mapLimit;y = mapLimit;


% colormap(gray)
% im.AlphaData = .2;
 imagesc(x,y,B)

cmap = [.5 .5 .5 %// light gray
        1  1  1]; %// white
colormap(cmap);
% colorbar('Ytick',[.25 .75],'Yticklabel',[0 1]) 
% colorbar
set(gca,'YDir','normal')  % otherwise the y axis is flipped
end


% %%%%% plot functions
% %{ Map plotting in 2D
% plot map (& UGV) in 2D
function plotAll(all)
% all = {targets,walls,recFur,cirFur,UGV};
targets = all{1}; walls =  all{2}; recFur = all{3}; cirFur = all{4}; UGV = all{5};
hold on
global mapLimit
% xlim([0  9.2])
% ylim([0  9.2])
xlim([mapLimit(1)  mapLimit(2)])
ylim([mapLimit(1)  mapLimit(2)])
axis equal

plotPtArr(targets,'k*')% plot all points

for i = 1:10
plotObjArr(walls,'k'); %plotting walls
plotObjArr(recFur,'c'); %plotting obs
end

plotObjArr(cirFur,'c'); %plotting obs

% plotRobot(UGV);
% drawFOV(UGV);

end

% helper for plotAll for points
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


%  backups

% %} Map plotting in 2D
% B2d = zeros(size(B3d{1}));
% for k = 1:length(B3d)
%     for i =  1:length(B3d{k})
%     for j = 1:length(B3d{k})
%         if ~isnan(B3d{k}(i,j))
%             B2d(i,j) = 1;
%         end
%     end
%     end
% end