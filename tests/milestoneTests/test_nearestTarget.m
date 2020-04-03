% Min. 8/8/2017
% Stage Goal: Load Cfree, Ctarget from mat. Compute PRM(narrow passage,
% EER) and generate milestones.

%{
Update: 

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
 figure;
 plotAll(all); 
 hold on 

% NOW TESTING
obstacles = {walls,recFur,cirFur};

global  A2d A3d

load ('A3d_150x40');
A2d = A3d(:,:,1); % 2d projection

% initilize variables
% I: not in C free    U: in C free  V: EER
I = zeros(size(A2d)); U = I; V = I;


 x = 7; y = 7;
 plot(x,y,'g*')
 nearID = nearestTar(targets,x,y) 
 foundTarget = targets(nearID);
 plot(foundTarget.loc(1),foundTarget.loc(2),'r*');
EER = EERcalc();

% END TESTING

%%%%%%%%% FUNCTIONS

% generate milestone, main function
function points = PRM()
global A2d mapLimit
Asize = size(A2d); n = Asize(1);% # of step of sampling

EER = 0.33;
for i = 1:n
    for j = 1:n
        % j indicates increment in x
        % loc = [myStart+(j-1)*step+step/2  myStart+(i-1)*step+step/2  currentTheta] ;
        if A2d(i,j) == 1 %  in C-free
            U(i,j) = 1; I(i,j) = 0; V(i,j)=EERcalc();   %$ function EER, determine current EER
        else
            U(i,j) = 0; I(i,j) = 1; V(i,j)=0;
        end
    end
end
U= U/sum(U(:));  V = V/sum(V(:));  
points = [];
end

% (EER functions
% calculate initial EER
function EER = EERcalc(i,j)
EER = 0;
B3d = load('B3d_150x40'); B3d = B3d.B3d;
B2d = zeros(size(B3d{1}));
for k = 1:length(B3d)
    for i =  1:length(B3d{k})
    for j = 1:length(B3d{k})
        if ~isnan(B3d{k}(i,j))
            B2d(i,j) = 1;
        end
    end
    end
end

if B2d(i,j) == 1 % in Ctarget
    EER = 0.2; % dummy value, EER of the nearest target
else
    EER = 0;
end

% for i =  1:length(B2d)
%     for j = 1:length(B2d)
%         if B2d(i,j) == 1 % in Ctarget
%             B2d(i,j) = 1;
%         end
%     end
% end
% figure
% plotCTarget(B2d);
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

% )EER functions


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
% %} Map plotting in 2D
