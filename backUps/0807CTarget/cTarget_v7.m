
% Min. 7/26/2017

%{
Update: 
1. optimize ifLineCross (cutLine) in cirObs  runtime improved


Notes:
Note that the target under the table is omitted

%}

% clear all;
addpath('/Users/min/Google Drive/pathPlanning(1)/infoData')
addpath('/Users/min/Google Drive/pathPlanning(1)/classdef')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\infoData')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\classdef')
%%%%%%%% MAIN
 [targets,walls,recFur,cirFur,UGV] = buildAll();


% NOW TESTING
obstacles = {walls,recFur,cirFur};
A = CFree(obstacles);
plotCTarget(A); % it is Cfree

% B = CTarget(targets,obstacles);
% plotCTarget(B);
% 
hold on


hold on
%  B = CTarget(targets,obstacles);
% plotCTarget(B);
% plotCTarget(A); % it is Cfree
hold on
plotAll(targets,walls,recFur,cirFur,UGV);   % any chance to modify this?
alpha(0.3)
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
n = 300;  % # of step of sampling
myStart = 0;
myEnd = 9.2;
step = (myEnd-myStart)/n;
xs = linspace(myStart,myEnd,n);
ys = linspace(myStart,myEnd,n);
A = zeros(n,n);

% build a test robot
fov = [0.3  0.8  1.0472];
loc = [0   0   pi];
size = [0.12  0.1];
tester = robot(loc,fov,size);
for i = 1:length(xs)    
    for j = 1:length(ys)
        tester.loc = [myStart+(j-1)*step+step/2  myStart+(i-1)*step+step/2  tester.loc(3)] ;  % j indicates increment in x
        if inCFree(tester,obstacles)
            A(i,j) = 1;
        end
    end
end
end

% generate a C-target distribution in the worldSpace
function B = CTarget(targets,obstacles)
H = load('Cfree'); A = H.A;
n = length(A);% # of step of sampling
m = 10; % step of theta
myStart = 0;
myEnd = 9.2;
step = (myEnd-myStart)/n;
xs = linspace(myStart,myEnd,n);
ys = linspace(myStart,myEnd,n);
ts = linspace(0,2*pi,m);
B = zeros(n,n);

% build a test robot
fov = [0.3  0.8  1.0472];
loc = [0   0   pi];
size = [0.1  0.12];
tester = robot(loc,fov,size);
for i = 1:length(xs)    
    for j = 1:length(ys)
        tester.loc = [myStart+(j-1)*step+step/2  myStart+(i-1)*step+step/2  tester.loc(3)] ;  % j indicates increment in x
        if A(i,j)== 1 % if not in C-free, not in C-target           
            if inCTarget(tester,targets,obstacles)
            B(i,j) = 1;
            end
        end
    end
end
end


% % plot functions

% plot all elements, such as targets, obstacles
function plotAll(targets,walls,recFur,cirFur,UGV)

figure(1)
hold on
xlim([0  9.2])
ylim([0  9.2])
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
% 2D
function plotCTarget(B)
x = [0 9.2];
y = [0 9.2];
% colormap(gray)
% im.AlphaData = .2;
 imagesc(x,y,B)

cmap = [.3 .3 .3 %// light gray
        1  1  1] %// white
colormap(cmap)
% colorbar('Ytick',[.25 .75],'Yticklabel',[0 1]) 
% colorbar
set(gca,'YDir','normal')  % otherwise the y axis is flipped
end
