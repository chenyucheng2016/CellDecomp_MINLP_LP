% Min. 7/31/2017
% version after CTarget_zoomInDemo;  
%{
Update: 
1. integrate 3D plotting functions


ToDos:
2. fix the target under table problem
1. plot3Dworld: rotated recFur has problem

Notes:
The target under the table is omitted

%}

% clear all; close all;
% addpath('/Users/min/Google Drive/pathPlanning(1)/infoData')
% addpath('/Users/min/Google Drive/pathPlanning(1)/classdef')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\infoData')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\classdef')
%%%%%%%% MAIN
 [targets,walls,recFur,cirFur,UGV] = buildAll();
 all = {targets,walls,recFur,cirFur,UGV};
global mapLimit   % setting for canvas size 
 mapLimit = [0  9.2];

% NOW TESTING
obstacles = {walls,recFur,cirFur};
global  A2d A3d

% % C-free
% A3d = CFree(obstacles);
% A2d = A3d(:,:,1); % 2d projection


% % 2D plot C target
% figure;
% plotCTarget(A2d); % 2d C free
% B2d = CTarget2d(targets,obstacles);
% plotCTarget(B2d);  % 2d C-Target
% hold on
% plotAll(all); 
% title('2D projection of C-target')
% alpha(0.6)
% xlim([3  5]); ylim([1  3]);

% 3D plot C target
figure;
% B3d = CTarget(targets,obstacles);

plotCTarget3d(B3d,all);



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


% build UGV
fov = [0.3  0.8  1.0472];
loc = [4.5906  4.5842  pi];
size = [0.12  0.1];
UGV = robot(loc,fov,size);


varargout = {targets,walls,recFur,cirFur,UGV};

end

% generate a C-target distribution in the worldSpace
function A = CFree(obstacles)
global mapLimit
n = 60;  % # of step of sampling
m = 20; % theta steps

myStart = mapLimit(1);
myEnd = mapLimit(2);

step = (myEnd-myStart)/n;
xs = linspace(myStart,myEnd,n);
ys = linspace(myStart,myEnd,n);
A = zeros(n,n,m);
tempA = zeros(n,n);
% ts = linspace(0,2*pi,m+1); % theta 

% build a test robot
fov = [0.3  0.8  pi/4];
loc = [mapLimit(1)   mapLimit(1)   pi];
size = [0.12  0.1];
tester = robot(loc,fov,size);
% for k = 1:length(ts)-1
%     currentTheta = ts(k);
currentTheta = pi;
for i = 1:length(xs)    
    for j = 1:length(ys)
        tester.loc = [myStart+(j-1)*step+step/2  myStart+(i-1)*step+step/2  currentTheta] ;  % j indicates increment in x
        if inCFree(tester,obstacles)
%             A(i,j,k) = 1;
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
n = length(A);% # of step of sampling
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

plotPtArr(targets,'k*')% plot all points

for i = 1:10
plotObjArr(walls,'k'); %plotting walls
plotObjArr(recFur,'c'); %plotting obs
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
% 2D Ctarget
function plotCTarget(B)
global mapLimit
x = mapLimit;y = mapLimit;


% colormap(gray)
% im.AlphaData = .2;
 imagesc(x,y,B)

cmap = [.3 .3 .3 %// light gray
        1  1  1]; %// white
colormap(cmap);
% colorbar('Ytick',[.25 .75],'Yticklabel',[0 1]) 
% colorbar
set(gca,'YDir','normal')  % otherwise the y axis is flipped
end

% 3D C-target
function plotCTarget3d(B,all)
hold on
s = B{1};
global mapLimit
xs = linspace(mapLimit(1),mapLimit(2),length(s));
for i =1:length(B)
    s = surf(xs,xs,B{i});

set(s, 'FaceAlpha',0.8, 'EdgeAlpha', 0.5);
%     s.EdgeColor = 'none';'FaceColor',[rand rand rand], 
    
end
hold on

plot3Dworld(all);  % plot target and wall

% %plot settings
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

% plot all objects, but rotated recFur has problem
function plot3Dworld(all)
targets = all{1}; walls =  all{2}; recFur = all{3}; cirFur = all{4}; UGV = all{5};
hold on
global mapLimit
% xlim([0  9.2])
% ylim([0  9.2])
xlim([mapLimit(1)  mapLimit(2)])
ylim([mapLimit(1)  mapLimit(2)])
axis equal
for x = targets
    plot3(x.loc(1),x.loc(2),-pi,'r*')% plot target points
end
for wall = walls
    myWall = surf( wall.vertices(1,:), wall.vertices(2,:), -pi*ones(length(wall.vertices(2,:))),[0.9 0.9] );
    set(myWall, 'FaceAlpha',0.3,'FaceColor','k');
end

% rotating one
% rotFur = recFur(end);
% cX = sum(rotFur.vertices(1,1:4))/4;  cY = sum(rotFur.vertices(2,1:4))/4; dx = 1.1/2; dy = 1/2;
% newVert = [cX-dx  cX-dx  cX+dx  cX+dx  cX-dx;cY-dy  cY+dy  cY+dy  cY-dy  cY-dy];
% h = surf(newVert(1,:), newVert(2,:), -pi*ones(5));
%  set(h, 'FaceAlpha',0.3,'FaceColor','k');
% direction = [0 0 1];
% rotate(h,direction,45);

newVert =[7.6370    7.6370    8.7370    8.7370    7.6370;
    0.6740    1.6740    1.6740    0.6740    0.6740];
h = surf(newVert(1,:),newVert(2,:), -pi*ones(5));
% direction = [0 0 1];
% rotate(h,direction,45)
% hold on 
set(h, 'FaceAlpha',0.3,'FaceColor','k');
% surf(newVert(1,:),newVert(2,:), -pi*ones(5));

recFur = recFur(1:end-1);
for obs = recFur
    myObs = surf(obs.vertices(1,:), obs.vertices(2,:), -pi*ones(length(obs.vertices(2,:))),[0.9 0.9] );
    set(myObs, 'FaceAlpha',0.3,'FaceColor','k');
    
end


for obs = cirFur
    r = obs.rad; xp = obs.loc(1); yp = obs.loc(2);
    t = linspace(0,2*pi);
    Xp = r*sin(t)+xp;
    Yp = r*cos(t)+yp;
 plot3(Xp,Yp,-pi*ones(length(Xp),length(Yp)));

end

% plotObjArr(cirFur,'c'); %plotting obs
% 

end