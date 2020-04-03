% Min. 8/15/2017
% Stage Goal: Load Cfree, Ctarget from mat. Compute PRM(narrow passage,
% EER) and generate milestones.

%{
Update: 
1. now have milestone class
2. now can generate milestone with simple split 
3. now can change color with the circle too!!

ToDos:
1. calculate EER for each point

Notes:

%}

% clear all; 
% close all;
% addpath('/Users/min/Google Drive/pathPlanning(1)/infoData')
% addpath('/Users/min/Google Drive/pathPlanning(1)/classdef')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\infoData')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\classdef')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\helperFuns')
warning off

%%%%%%%% MAIN
[targets,walls,recFur,cirFur,UGV] = buildAll();


global mapLimit   % setting for canvas size 
mapLimit = [0  9.2];
global  A2d A3d B3d  B2d


% NOW TESTING
% A3d = CFree(UGV,obstacles);
% A3d = load('A3d_300x60'); A3d = A3d.A3d;
% A2d = A3d(:,:,1); % 2d projection
% B3d = load('B3d_300x60');  B3d = B3d.B3d;
% obstacles = {walls,recFur,cirFur};
% B3dtoB2d();
% 
% 
% ms = testMS(targets);
% all = {targets,walls,recFur,cirFur,UGV,ms};
% 
% figure;
% plotCTarget(B2d);
% hold on
% plotAll(all); 
% hold on 
[i,j] = xy2grid(ms.loc(1),ms.loc(2));
eer = Vcalc(targets,i,j)
% msUnif = [msUnif  milestone([ms.loc(1)  ms.loc(2)],'uniform',eer)];
% END TESTING

%%%%%%%%% FUNCTIONS

% generate milestone, main function

function ms = testMS(targets)
ms = [];
[x1,y1] = xy2grid(7.046,7.077);
eer = Vcalc(targets,x1,y1);
ms = [ms  milestone([7.046,7.077],'1',eer)];


[x1,y1] = xy2grid(7.169,6.892);
eer = Vcalc(targets,x1,y1);
ms = [ms  milestone([7.169,6.892],'2',eer)];


[x1,y1] = xy2grid(6.154,7.446);
eer = Vcalc(targets,x1,y1);
ms = [ms  milestone([6.154,7.446],'3',eer)];

[x1,y1] = xy2grid(6.585,6.769);
eer = Vcalc(targets,x1,y1);
ms = [ms  milestone([6.585,6.769],'4',eer)];

end


function ms = genMS(targets)
global A2d
n = length(A2d);
numMS = 150;  % number of total milestone
w = 0.8;  % higher  less RBB
nUnif = round(numMS * w);
nRBB = numMS - nUnif;
sigma = round(n*0.08);
msRBB = [];   msUnif = [];
maxLoop = 20; % need to abandon a point if dead loop
step = 10;  % r for orthogonal test for RBB
while length(msRBB) < nRBB || length(msUnif) < nUnif
    x1 =  randi(n);
    y1 =  randi(n);
    
    % in obstacle?
    if A2d(x1,y1) == 1 %  in C-free
        if length(msUnif) < nUnif
            [x,y] = grid2xy(x1,y1);
            eer = Vcalc(targets,x1,y1);
            msUnif = [msUnif  milestone([x  y],'uniform',eer)];
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
                    eer = Vcalc(targets,x1,y1);
                    [x_rbb,y_rbb] = grid2xy(xq,yq);
                    msRBB = [msRBB  milestone([x_rbb,y_rbb],'RBB',eer)];
                    success = true;
                end
            end
            count = count+1;
        end
        
    end
    
end
ms = [msUnif  msRBB];
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
function V = Vcalc(targets,i,j)
global B2d
if B2d(i,j) == 1 % in Ctarget
    [x,y] = grid2xy(i,j);
    nearestID = nearestTar(targets,x,y);
    V = EERcalc(targets(nearestID));
else
    V = 0;
end

end

function [i,j] = xy2grid(x,y)
global A2d mapLimit 
Asize = size(A2d); n = Asize(1);% # of step of sampling
myStart = mapLimit(1);
myEnd = mapLimit(2);

step = (myEnd-myStart)/n;

i = floor((y-myStart)/step)+1;
j = floor((x-myStart)/step)+1;
if i > 300
    i = 300;
end
if j > 300
    j = 300;
end
end

function [x,y] = grid2xy(i,j)
global mapLimit B3d A2d
 n = length(A2d);% m = length(B3d{1});
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



% %%%%% plot functions
% %{ Map plotting in 2D
% plot map (& UGV) in 2D
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

plotPtArr(targets,'k*')% plot all points
% for myMS = ms
%     plotPtArr(myMS,'b^')% plot all milestones
% end
plotMS(ms); 

for i = 1:10
plotObjArr(walls,'k'); %plotting walls
plotObjArr(recFur,[0.5  1  1]); %plotting obs
end

plotObjArr(cirFur,[0.5  1  1]); %plotting obs

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

% %} Map plotting in 2D
function B3dtoB2d()
global B3d B2d
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
end

function plotMS(ms)
hold on
allEER = [];
colors = [[0.6  0.2  0.4];[0.6   0.7  0];[0.8  0.6  0.3];[1  0.5  0.7]];
for myMS = ms
    allEER = [allEER  myMS.EER];
end
A = unique(allEER);
count = 1;
for eer = A
%     color = rand(1,3);
if eer ~= 0
    color = colors(count,:);
else
    color = [0.9  0.8  0.6];
end
    xs = []; ys = [];
    for myMS = ms
        if myMS.EER == eer
            xs = [xs   myMS.loc(1)];
            ys = [ys   myMS.loc(2)];
        end
    end
h(count) = plot(xs,ys,'^','MarkerFaceColor',color,'MarkerEdgeColor','None');
count =  count + 1;

end
str = strtrim(cellstr(strcat('EER = ',num2str(A.',3))));
legend(h(:),str{:}); 
end