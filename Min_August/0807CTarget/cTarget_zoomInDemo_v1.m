
% Min. 7/26/2017

%{
Update: 
Now can plot 3d C-target: x y theta

Notes:
Note that the target under the table is omitted

%}

 clear all; close all;
% addpath('/Users/min/Google Drive/pathPlanning(1)/infoData')
% addpath('/Users/min/Google Drive/pathPlanning(1)/classdef')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\infoData')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\classdef')
%%%%%%%% MAIN
 [targets,walls,recFur,cirFur,UGV] = buildAll();
 all = {targets,walls,recFur,cirFur,UGV};
global mapLimit
 mapLimit = [0  5];

% NOW TESTING
obstacles = {walls,recFur,cirFur};
global  A2d A3d

% % C-free
A3d = CFree(UGV,obstacles);
A2d = A3d(:,:,1); % 2d projection

% % 2D plot C target
figure;
plotCTarget(A2d); % 2d C free
B2d = CTarget2d(targets,obstacles);
plotCTarget(B2d);  % 2d C-Target
hold on
plotAll(all); 
title('2D projection of C-target')
alpha(0.6)
xlim([3  5]); ylim([1  3]);
% % 3D plot C target
figure;
 B3d = CTarget(targets,obstacles);

 plotCTarget3d(B3d,all);
xlim([3   5]);ylim([1   3]);

% END TESTING


%%%%%%%%% FUNCTIONS

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
recFur = recObs([4.1  4.1  4.3   4.3;2.3  2.5  2.5  2.3],'furniture');


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
n = 400;  % # of step of sampling
m = 40; % theta steps

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
step = (myEnd-myStart)/n;
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
            tester.loc = [myStart+(j-1)*step+step/2  myStart+(i-1)*step+step/2  currentTheta] ;  
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



% 2D generate a C-target distribution in the worldSpace
function B = CTarget2d(targets,obstacles)
global A2d mapLimit
% H = load('Cfree'); A = H.A;
A = A2d;
n = length(A2d);% # of step of sampling
myStart = mapLimit(1);
myEnd = mapLimit(2);
step = (myEnd-myStart)/n;
xs = linspace(myStart,myEnd,n);
ys = linspace(myStart,myEnd,n);


B = zeros(n,n);

% build a test robot
fov = [0.3  0.8  pi/4];
loc = [0   0   pi];
size = [0.1  0.12];
tester = robot(loc,fov,size);
for i = 1:length(xs)    
    for j = 1:length(ys)
        tester.loc = [myStart+(j-1)*step+step/2  myStart+(i-1)*step+step/2  tester.loc(3)] ;  % j indicates increment in x
        if A(i,j)== 1 % if not in C-free, not in C-target           
            if inCTarget2d(tester,targets,obstacles)
            B(i,j) = 1;
            end
        end
    end
end
end



% % plot functions

% plot all elements, such as targets, obstacles
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

plotPtArr(targets,'r*')% plot all points

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
function plotCTarget(B)
global mapLimit
x = mapLimit;y = mapLimit;
% x = [0 9.2];
% y = [0 9.2];
% colormap(gray)
% im.AlphaData = .2;
 imagesc(x,y,B)

cmap = [.3 .3 .3 %// light gray
        1  1  1];%// white
colormap(cmap)
% colorbar('Ytick',[.25 .75],'Yticklabel',[0 1]) 
% colorbar
set(gca,'YDir','normal')  % otherwise the y axis is flipped
end

function plotCTarget3d(B,all)
hold on
s = B{1};
global mapLimit
xs = linspace(mapLimit(1),mapLimit(2),length(s));
% cs = linspace(0,1,length(B));
for i =1:length(B)
%     matrix = B{i}*9.2/150;
%     surf(matrix);
    s = surf(xs,xs,B{i});

set(s, 'FaceAlpha',0.8, 'EdgeAlpha', 0.5);
%     s.EdgeColor = 'none';'FaceColor',[rand rand rand], 
    
end
hold on
% plotAll(all); % plot target and wall
plot3Dworld(all);
% axis equal
% title('3D C-target')
xlabel('x(m)')
ylabel('y(m)')
zlabel('\theta (rad)')
colorbar
colorbar('Ticks',[-pi -pi/2 0 pi/2 pi],...
         'TickLabels',{'-\pi','-\pi/2','0','\pi/2','pi',})
zticks([-pi -pi/2 0 pi/2 pi])
zticklabels({'-\pi','-\pi/2','0','\pi/2','pi',})
set(gca,'FontSize',15)
grid on
hold on
end

function plot3Dworld(all)
targets = all{1}; walls =  all{2}; recFur = all{3}; cirFur = all{4}; UGV = all{5};
hold on
global mapLimit
% xlim([0  9.2])
% ylim([0  9.2])
xlim([mapLimit(1)  mapLimit(2)])
ylim([mapLimit(1)  mapLimit(2)])
axis equal

plot3(targets.loc(1),targets.loc(2),-pi,'r*')% plot target points
wall = surf( walls.vertices(1,:), walls.vertices(2,:), -pi*ones(length(walls.vertices(2,:))),[0.9 0.9] );
set(wall, 'FaceAlpha',0.3,'FaceColor','k');
obs = surf( recFur.vertices(1,:), recFur.vertices(2,:), -pi*ones(length(recFur.vertices(2,:))),[0.9 0.9] );
set(obs, 'FaceAlpha',0.3,'FaceColor','k');

% plotObjArr(cirFur,'c'); %plotting obs
% 

end
