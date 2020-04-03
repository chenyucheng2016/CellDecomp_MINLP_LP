
% Min. 7/24/2017

%{
Update: 
2. add in-obstacles for recObs and cirObs
3. for every point not in obstacle(C-free) check if in FOV


Notes:
modify FOV directrion in functino CTarget, tester = robot(loc,fov,size); 




Problem:
cannot combine cirFur and recFur
can't adjust color  by [1,0,1]
can't change alpha on cirObs

%}
% 
% clear all;
%  close all;
% addpath('C:\Users\Ferrari Lab\Desktop\pathPlanning\classdef')  % add folder of class 
% addpath('C:\Users\Ferrari Lab\Desktop\pathPlanning\infoData')
% addpath('/Users/min/Google Drive/pathPlanning/infoData')
% addpath('/Users/min/Google Drive/pathPlanning/classdef')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\infoData')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\classdef')
%%%%%%%% MAIN
 [targets,walls,cirFur,UGV] = buildAll();


% % NOW TESTING
 obstacles = {walls,cirFur};
%  A = CFree(obstacles);
% % % figure(2)
% % figure(3)
% plotCTarget(A); % it is Cfree
% % B = CTarget(targets,obstacles);
% % plotCTarget(B);
% 

% hold on
% plotAll(targets,walls,cirFur,UGV);   % any chance to modify this?
% hold on

% build a test robot
fov = [0.3  0.8  1.0472];
loc = [6.705   8.108   pi];
size = [0.12  0.1];
tester = robot(loc,fov,size);
tf = inCFree(tester,obstacles)
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
walls = recObs([1  2  2  1;1  1  2  2],'wall');


% build recFur
% load('recFurCor.mat');
% recFur = recObs(recFurCor{8},'furniture');


% build cirFur
load('cirFurCor.mat');
cirFur = cirObs(cirFurCor{2}(1:2),cirFurCor{2}(3),'furniture');



% build UGV
fov = [0.3  0.8  1.0472];
loc = [4.5906  4.5842  pi];
size = [0.12  0.1];
UGV = robot(loc,fov,size);


varargout = {targets,walls,cirFur,UGV};

end

% generate a C-target distribution in the worldSpace
function A = CFree(obstacles)
n = 50;  % # of step of sampling
size = [0.1  0.12]; % robot size
myStart = 0;
myEnd = 3;
% stepX = (myEnd-myStart-size(1))/n;
% stepY = (myEnd-myStart-size(2))/n;
% xs = linspace(myStart+size(1)/2,myEnd-size(1)/2,n);
% ys = linspace(myStart+size(2)/2,myEnd-size(2)/2,n);
xs = linspace(myStart,myEnd,n+1);
ys = linspace(myStart,myEnd,n+1);
stepX = xs(2)-xs(1);stepY = ys(2)-ys(1);
A = zeros(n,n);

% build a test robot
fov = [0.3  0.8  1.0472];
loc = [0   0   pi];
size = [0.1  0.12];
tester = robot(loc,fov,size);
for i = 1:n
%     tester.loc(2) = tester.loc(2)+stepY;
    for j = 1:n
        tester.loc = [(xs(j)+xs(j+1))/2   (ys(i)+ys(i+1))/2  tester.loc(3)] ;  % j indicates increment in x
        if inCFree(tester,obstacles)
            A(i,j) = 1;
        end
    end
end
end


% generate a C-target distribution in the worldSpace
function A = CFree2(obstacles)
n = 60;  % # of step of sampling
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
        tester.loc = [myStart+(j-1)*step+step/2  myStart+(i-1)*step+step/2  tester.loc(3)] ;  % j indicates increment in x
        if inCFree(tester,obstacles)
            A(i,j) = 1;
        end
    end
end
end


% generate a C-target distribution in the worldSpace
function B = CTarget(targets,obstacles)
n = 40;  % # of step of sampling
myStart = 0;
myEnd = 9.2;
step = (myEnd-myStart)/n;
xs = linspace(myStart,myEnd,n);
ys = linspace(myStart,myEnd,n);
B = zeros(n,n);

% build a test robot
fov = [0.3  0.8  1.0472];
loc = [0   0   pi];
size = [0.1  0.12];
tester = robot(loc,fov,size);
for i = 1:length(xs)    
    for j = 1:length(ys)
        tester.loc = [myStart+(j-1)*step+step/2  myStart+(i-1)*step+step/2  tester.loc(3)] ;  % j indicates increment in x
        if inCTarget(tester,targets,obstacles)
            B(i,j) = 1;
        end
    end
end
end


% % plot functions

% plot all elements, such as targets, obstacles
function plotAll(targets,walls,cirFur,UGV)

figure(1)
hold on
xlim([0  3])
ylim([0  3])
axis equal

for i = 1:10
plotObjArr(walls,'k'); %plotting walls
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

function plotCTarget(B)
x = [0 3];
y = [0 3];
% colormap(gray)
% im.AlphaData = .2;
imagesc(x,y,B)

cmap = [.3 .3 .3 %// light gray
        1  1  1]; %// white
colormap(cmap)
% colorbar('Ytick',[.25 .75],'Yticklabel',[0 1]) 
% colorbar
set(gca,'YDir','normal')  % otherwise the y axis is flipped
end


