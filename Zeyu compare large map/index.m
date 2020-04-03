% Min. 10/11/2017
% 
%{
Update: 
1. plot in 2D.
2. 
Problem:

ToDos:
1. 

Notes:

%}

% clear all; 
% close all;
% addpath('/Users/min/Google Drive/pathPlanning(1)/infoData')legend 
% addpath('/Users/min/Google Drive/pathPlanning(1)/classdef')
% addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\infoData')
% addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\classdef')
% addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\helperFuns')
warning off

%%%%%%%% MAIN
global UGV
% [targets,walls,recFur,cirFur,UGV] = buildAll();
all = load('all.mat'); all = all.all;
obstacles = {all{2},all{3}};
targets = all{1};

global mapLimit   % setting for canvas size 
mapLimit = [0  9.2];

global  A2d A3d B3d  B2d 
A2d = load('A2d_300.mat');  A2d = A2d.A2d;
A3d = load('A3d_300x60.mat');  A3d = A3d.A3d;
B3d = load('B3d_300x60.mat');  B3d = B3d.B3d;
B3dtoB2d();

figure;
plotCTarget(B2d);
hold on

% plotAll(all); % hold on; % connectMap = connectMS(obstacles,ms);

% NOW TESTING
ms = genTwoUniform(targets,10,100); %nUnif(pink), nTarget(blue)

global msColor
msColor = true; % plot milestone in different color for each type
all{5} = UGV; all{6} = ms;
plotAll(all);

% END TESTING

%%%%%%%%% FUNCTIONS
function MS = sampleEER(A,N)
[n,~] = size(A);
inds =1 :n*n;
B = zeros(1,n*n);
for i = 1:n
    for j = 1:n
        ind =  sub2ind([n,n],i,j);
        B(ind) = A(i,j);
    end
end

Mss_ind = [];

% sampling and deal with non repeat
while length(Mss_ind) < N
    R = randsample(inds, 1, true, B);
%     disp(B(R));
    if isempty(find(Mss_ind==R, 1))  % if result does not have R, add R
        Mss_ind = [Mss_ind R];
    end
end

MS = [];
% map the ind back to 2D
for ms = Mss_ind
    [i,j] = ind2sub(size(A),ms);
%     fprintf('i = %d, j = %d \n',i,j);
    [x,y] = grid2xy(i,j);
    MS = [MS  milestone([x,y],'EER',A(i,j))];  % read EER val from C
end

end
% generate milestone in narraow passages & uniform
function ms = genTwoUniform(targets,nUnif, nTarget)
global A2d B2d
n = length(B2d);

msTarget = [];  msUnif = [];

    while length(msUnif) < nUnif
        x1 =  randi(n);
        y1 =  randi(n);

        if A2d(x1,y1) == 1 %  in C-free
            
                [x,y] = grid2xy(x1,y1);
    %             eer = Vcalc(targets,x1,y1);
                msUnif = [msUnif  milestone([x  y],'uniform',1)];
        

        end
    end
    
    while length(msTarget) < nTarget
        x1 =  randi(n);
        y1 =  randi(n);

        % in obstacle?
        if B2d(x1,y1) == 1 %  in C-free
           
                [x,y] = grid2xy(x1,y1);
    %             eer = Vcalc(targets,x1,y1);
                msTarget = [msTarget  milestone([x  y],'target',1)];
          

        end
    end
    
    ms = [msTarget  msUnif];

end


function C = C_EER(targets)
%  H = load('Cfree_zoom_v1'); A = H.A;
global A2d  
Asize = size(A2d);
n = Asize(1);% # of step of sampling
C = zeros(n,n);

for i = 1:n
    for j = 1:n
        C(i,j) = Vcalc(targets,i,j);
    end
end
end


% generate milestone with uniform + EER
function ms = genMS_unif_EER(targets,nRBB)
global A2d
n = length(A2d);

msUnif = []; 
while length(msRBB) < nRBB
    x1 =  randi(n);
    y1 =  randi(n);
    
    % in obstacle?
    if A2d(x1,y1) == 1 %  in C-free
        if length(msUnif) < nUnif
            [x,y] = grid2xy(x1,y1);
            eer = Vcalc(targets,x1,y1);
            msUnif = [msUnif  milestone([x  y],'uniform',eer)];
        end
    end
    
end
% UGV start point
mid = round(n/2);
eer = Vcalc(targets,mid, mid);
[x_mid,y_mid] = grid2xy(mid,mid);
ms = [milestone([x_mid,y_mid],'UGV',eer)    msRBB];
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

% for all milestone, check their connectivity
% weight = inf  if not connected; else = distance - endpoint.EER
function mtrx = connectMS(obstacles,ms)
p = 10;  % EER weight varaiable
maxDist = 4;
n = length(ms);
mtrx = Inf*ones(n);
for i = 1:n
    ms1 = ms(i);
    for j = 1:n
        ms2 = ms(j);
        if i==j 
            mtrx(i,j) = -ms1.EER;
            plot([ms1.loc(1)  ms2.loc(1)],[ms1.loc(2)  ms2.loc(2)],'k-');
            continue
        end
        dist = distPts(ms1.loc(1),ms1.loc(2),ms2.loc(1),ms2.loc(2));
        if ifConnect(ms1,ms2,obstacles) &&  dist < maxDist
            mtrx(i,j) = dist - p* ms2.EER;
            plot([ms1.loc(1)  ms2.loc(1)],[ms1.loc(2)  ms2.loc(2)],'k-');
        end
    end
end
end

%  precondition, pos in C-Target2D
% find all target IDs that it can see from this x,y (NOT TESTED)
function ids = idsCanSee(targets,x,y)
global UGV  obstacles
minDist = UGV.fov(1); maxDist = UGV.fov(2);
ids = [];
targetID = 0; dist = inf;
    for target = targets
        targetID = targetID+1;
        thisDist = distPts(target.loc(1),target.loc(2),x,y) ;
        % dummy milestone to test connectivity 
        tester = milestone([x,y],'test',0);
        if thisDist < maxDist  && thisDist > minDist && ifConnect(tester,target,obstacles)
            % make sure that target in FOV
            ids =[ids targetID];
        end
    end
end


% (EER (V) functions
% calculate initial EER modified as culmulated(NOT TESTED)
function V = Vcalc(targets,i,j)
global B2d
V = 0;
if B2d(i,j) == 1 % in Ctarget
    [x,y] = grid2xy(i,j);
    
    ids = idsCanSee(targets,x,y);
    for id = ids
        V = V + EERcalc(targets(id));
    end
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
global UGV
minDist = UGV.fov(1); maxDist = UGV.fov(2);
nearestID = 0;
targetID = 0; dist = inf;
    for target = targets
        targetID = targetID+1;
        thisDist = distPts(target.loc(1),target.loc(2),x,y) ;
        if thisDist < dist && thisDist < maxDist  && thisDist > minDist  % make sure that target in FOV
            nearestID = targetID;
            dist = thisDist;
        end
    end
    if nearestID == 0
        disp('error')
    end
end

% given the target (current cueLevel and final id) get oneStep EER
% final id is just for calculating id at current level.
function EER = EERcalc1(target) % 
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

% given the target (current cueLevel and final id) 
% reveal all cues EER(1/2/3 step EER)
% final id is just for calculating id at current level.
function EER = EERcalc(target) % 
oneStep = load('EER_oneStep');  oneStep = oneStep.oneStep;
twoStep = {0.0572,[0.3558,0]};
threeStep = 0.1847;
crt_cueLev = target.crt_cueLev; id = target.id;
if crt_cueLev == 3 % no more cues to reveal 
    EER = 0;
    return
else
    id_crt =  ceil(id/2^(3-crt_cueLev));
    if crt_cueLev == 2 % do one step EER
        EER = oneStep{crt_cueLev+1}(id_crt);
    elseif crt_cueLev == 1    % shape is known, do 2-step EER
        EER = twoStep{crt_cueLev+1}(id_crt);
    else % know no cues, do 3-step EER
        EER = threeStep;
    end
end
end

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
% generate a C-target distribution in the worldSpace
function A = CFree(robot,obstacles)
% build a tester circle zoom
w = robot.size(1); h = robot.size(2);
r = sqrt(w^2+h^2)/2;

% plotObs(robotCir,'r',true);

global mapLimit
n = 800;  % # of step of sampling
m = 1; % theta steps

myStart = mapLimit(1);
myEnd = mapLimit(2);

step = (myEnd-myStart)/(n-1);
xs = linspace(myStart,myEnd,n);
ys = linspace(myStart,myEnd,n);
A = zeros(n,n,m);
tempA = zeros(n,n);
% ts = linspace(0,2*pi,m+1); % theta 

for i = 1:length(xs)
    for j = 1:length(ys)
        robotCir = cirObs([myStart+(j-1)*step  myStart+(i-1)*step],r,'tester');
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

% 3D generate a C-target distribution in the worldSpace
function B = CTarget(targets,obstacles)
%  H = load('Cfree_zoom_v1'); A = H.A;
 global A3d  A2d  mapLimit
 A = A3d;
Asize = size(A3d);
n = Asize(1);% # of step of sampling
m = Asize(3); % step of theta
myStart = mapLimit(1);
myEnd = mapLimit(2);
step = (myEnd-myStart)/(n-1);
xs = linspace(myStart,myEnd,n);
ys = linspace(myStart,myEnd,n);
% ts = linspace(0,2*pi,m+1); % theta 
ts = linspace(-pi,pi,m+1); % theta 
% B = zeros(n,n,m);
B = cell(1,m);   % store m layers

% build a test robot
fov = [0.3  0.8  pi/4];
loc = [0   0   pi];
robotSize = [0.1  0.12];
tester = robot(loc,fov,robotSize);
for k = 1:length(ts)-1
    currentTheta = ts(k);
    tempB = NaN(n,n);
    for i = 1:length(xs)
        for j = 1:length(ys)
            % j indicates increment in x
            tester.loc = [myStart+(j-1)*step  myStart+(i-1)*step  currentTheta] ;  
            if A(i,j,m) == 1 % if not in C-free, not in C-target
                if inCTarget(tester,targets,obstacles)
                    tempB(i,j) = currentTheta;
                end
            end
        end
    end
    B{k} = tempB;
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



plotPtArr(targets,'k*')% plot all points
% for myMS = ms
%     plotPtArr(myMS,'b^')% plot all milestones
% end

counter = 1;
while counter < 5
plotMS(ms); 
counter = counter + 1;
end
% plotMS(ms); 
for i = 1:10
plotObjArr(walls,'k'); %plotting walls
plotObjArr(recFur,[0.5  1  1]); %plotting obs
end

plotObjArr(cirFur,[0.5  1  1]); %plotting obs

% plotRobot(UGV);
% drawFOV(UGV);
axis equal
xlim([mapLimit(1)  mapLimit(2)])
ylim([mapLimit(1)  mapLimit(2)])

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

cmap = [.8 .8 .8 %// light gray
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

function plotMS(mss)
hold on
colors = [[0.1  0.2  0.7];[0.8  0.2  0.9];[0.9  0.9  0.9];[0.5  0.2  0.3]];
global msColor
for ms = mss
    if msColor == true  % plot in different color   
        if strcmp(ms.type,'target') 
            scatter(ms.loc(1),ms.loc(2),'^','MarkerFaceColor',colors(1,:),'MarkerEdgeColor','none');
        elseif strcmp(ms.type,'uniform') 
            scatter(ms.loc(1),ms.loc(2),'^','MarkerFaceColor',colors(2,:),'MarkerEdgeColor','none');
        elseif strcmp(ms.type,'EER') 
            scatter(ms.loc(1),ms.loc(2),'^','MarkerFaceColor',colors(3,:),'MarkerEdgeColor','none');
        end
    else   % plot ms in same color
%         scatter(ms.loc(1),ms.loc(2),'^','MarkerFaceColor','k','MarkerEdgeColor','none');
        scatter(ms.loc(1),ms.loc(2),'^','MarkerFaceColor','None','MarkerEdgeColor','b');
    end
end
end


function plotEERmap(C,all)
global mapLimit  
figure;
x = mapLimit;y = mapLimit;

% colormap(gray)
% im.AlphaData = .2;
 imagesc(x,y,C)
% cm = load('cm'); 
% cm = cm.mycmap;
% colormap(cm);
% cmap = [.5 .5 .5 %// light gray
%         1  1  1]; %// white
% colormap(pink);
% colorbar('Ytick',[.25 .75],'Yticklabel',[0 1]) 
% colorbar
set(gca,'YDir','normal')  % otherwise the y axis is flipped

% n = length(C);
% xs = linspace(mapLimit(1),mapLimit(2),n);
% s = surf(xs,xs,C);
% set(s,'EdgeColor','none');
 colormap(flipud(pink));
colormapeditor
colorbar
hcb=colorbar;
title(hcb,'EER');
hold on
axis equal
plotAll(all);
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


% %{ Map plotting in 3D
% plot all objects (map) in 3D
function plot3Dworld(all,dZ)
targets = all{1}; walls =  all{2}; recFur = all{3}; cirFur = all{4}; UGV = all{5}; mss = all{6};
hold on
global mapLimit
% xlim([0  9.2])
% ylim([0  9.2])
xlim([mapLimit(1)  mapLimit(2)])
ylim([mapLimit(1)  mapLimit(2)])
axis equal



colors = [[0  0  0.1];[0  0.5  0.1];[0.4  0  0.7];[0.6  0.7  0.9];[0  0.2  0.3]];
eers = [0,0.0317,0.05932,0.4633];
for x = targets
     scatter3(x.loc(1),x.loc(2),dZ,'k*');
    % to plot in different color
%     for k = 1:length(eers)
%         if(abs( EERcalc(x) - eers(k)) < 0.0001)
% %             temp_ps = [ps(k)  [x.loc(1)  x.loc(2)]];
% %             fprintf('location: %f, %f;  eer: %f  \n',x.loc(1),x.loc(2),eers(k));
%             scatter3(x.loc(1),x.loc(2),3,'*','MarkerEdgeColor',colors(k,:));% plot target points
%             break
%         end
%     end
    
end
% %put in console to get legend
% figure();colors = [[0  0  0.1];[0  0.5  0.1];[0.4  0  0.7];[0.6  0.7  0.9];[0  0.2  0.3]];
% scatter3(1,1,1,'MarkerFaceColor',colors(1,:),'MarkerEdgeColor','none');hold on
% scatter3(1,2,1,'MarkerFaceColor',colors(2,:),'MarkerEdgeColor','none');% plot target points
% scatter3(1,3,1,'MarkerFaceColor',colors(3,:),'MarkerEdgeColor','none');% plot target points
% scatter3(1,4,1,'MarkerFaceColor',colors(4,:),'MarkerEdgeColor','none');% plot target points
% legend('0','0.0317','0.05932','0.4633');



for wall = walls
    myWall = surf( wall.vertices(1,:), wall.vertices(2,:), dZ*ones(length(wall.vertices(2,:))),[0.9 0.9] );
    set(myWall, 'FaceAlpha',0.3,'FaceColor','k');
end

% rotating one
newVert =[8.0550    7.4463    8.3190    8.9277    8.0550;
    0.4425    1.2359    1.9055    1.1121    0.4425];
h = fill3(newVert(1,:),newVert(2,:),dZ*ones(5),'k');
set(h, 'FaceAlpha',0.15,'FaceColor','k');

recFur = recFur(1:end-1);
for obs = recFur
    myObs = surf(obs.vertices(1,:), obs.vertices(2,:), dZ*ones(length(obs.vertices(2,:))),[0.9 0.9] );
    set(myObs, 'FaceAlpha',0.3,'FaceColor','k');
    
end


for obs = cirFur
    r = obs.rad; xp = obs.loc(1); yp = obs.loc(2);
    plotCircle3d(r,xp,yp,dZ);
%     t = linspace(0,2*pi);
%     Xp = r*sin(t)+xp;
%     Yp = r*cos(t)+yp;
%  plot3(Xp,Yp,-pi*ones(length(Xp),length(Yp)));



end

    % to plot in different color
%     for k = 1:length(eers)
%         if(abs( EERcalc(x) - eers(k)) < 0.0001)
% %             temp_ps = [ps(k)  [x.loc(1)  x.loc(2)]];
% %             fprintf('location: %f, %f;  eer: %f  \n',x.loc(1),x.loc(2),eers(k));
%             scatter3(x.loc(1),x.loc(2),dZ,'MarkerFaceColor',colors(k,:),'MarkerEdgeColor','none');% plot target points
%             break
%         end
%     end



colors = [[0  0.5  0.1];[0.4  0  0.7];[0.6  0.7  0.9];[0  0.2  0.3]];

for ms = mss
    scatter3(ms.loc(1),ms.loc(2),3,'^','MarkerFaceColor',colors(1,:),'MarkerEdgeColor','none');
%     if strcmp(ms.type,'RBB') 
%         scatter3(ms.loc(1),ms.loc(2),3,'^','MarkerFaceColor',colors(1,:),'MarkerEdgeColor','none');
%     elseif strcmp(ms.type,'uniform') 
%         scatter3(ms.loc(1),ms.loc(2),3,'^','MarkerFaceColor',colors(2,:),'MarkerEdgeColor','none');
%     elseif strcmp(ms.type,'EER') 
%         scatter3(ms.loc(1),ms.loc(2),3,'^','MarkerFaceColor',colors(3,:),'MarkerEdgeColor','none');
% 
%     end
end


% %legend
% figure();colors = [[0  0.5  0.1];[0.4  0  0.7];[0.6  0.7  0.9];[0  0.2  0.3]];
% scatter3(1,1,1,'MarkerFaceColor',colors(1,:),'MarkerEdgeColor','none');hold on
% scatter3(1,2,1,'MarkerFaceColor',colors(2,:),'MarkerEdgeColor','none');% plot target points
% scatter3(1,3,1,'MarkerFaceColor',colors(3,:),'MarkerEdgeColor','none');% plot target points
% legend('RBB','uniform','EER');

% plotObjArr(cirFur,'c'); %plotting obs
% 

end

% helper for plot3Dworld
function plotCircle3d(r,cX,cY,dZ)
theta = 0:0.01:2*pi;
h = 0.01;
x = r*cos(theta)+cX;
y = r*sin(theta)+cY;

y(end) = cY;

z1 = dZ;

h = patch(x,y,z1*ones(size(x)),'k');
set(h, 'FaceAlpha',0.6);
end