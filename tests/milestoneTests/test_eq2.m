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
all = {targets,walls,recFur,cirFur,UGV};

global mapLimit A3d A2d  % setting for canvas size 
 mapLimit = [0  5];
%  figure;
%  plotAll(all); 
%  hold on 
 
 % % C-free
 obstacles = {walls,recFur,cirFur};
% A3d = CFree(UGV,obstacles);
% A3d = load('test_A3d_400x40'); A3d = A3d.A3d;
% A3d = load('test_A3d_120x2'); A3d = A3d.A3d;
A3d = load('test_A3d_15x1'); A3d = A3d.A3d;
 A2d = A3d(:,:,1); % 2d projection

% % 2D plot C target
% figure;
% plotCTarget(A2d); % 2d C free
% hold all
%  plotAll(all); 


[points  B  U  I  V]= PRM(targets);

 % generate milestone, main function
function [points  B  U  I  V] = PRM(targets)
global A2d mapLimit B3d A3d
Asize = size(A2d); n = Asize(1);% # of step of sampling
m = length(A3d);

% initilize variables
% I: not in C free    U: in C free  V: EER
I = zeros(n,n); U = I; V = I; f =  cell(n);


for i = 1:n
    for j = 1:n
        % j indicates increment in x
        % loc = [myStart+(j-1)*step+step/2  myStart+(i-1)*step+step/2  currentTheta] ;
        if A2d(i,j) == 1 %  in C-free
            U(i,j) = 1; I(i,j) = 0;   V(i,j)=0.1; % dummy value
        else
            U(i,j) = 0; I(i,j) = 1; V(i,j)=0;
        end
    end
end


for i = 1:n
    for j = 1:n
         % for q0 inCB
        Z = 0;
        for iCB = 1:n
            for jCB = 1:n 
                [x,y] = grid2xy(iCB,jCB);
                temp = I(iCB,jCB)*gaussian2d(x,y);  %% might be incorrect
                A{iCB,jCB} = temp;
                Z = Z+temp;
            end
        end
        for iCB = 1:n
            for jCB = 1:n
                if temp ~= 0
                A{iCB,jCB} = A{iCB,jCB}/temp ;
                end 
            end
        end
        
        B{i,j} = A;
    end
end




% for k = 1:m
% for i = 1:n
%     for j = 1:n
%         % j indicates increment in x
%         % loc = [myStart+(j-1)*step+step/2  myStart+(i-1)*step+step/2  currentTheta] ;
%         if A3d(i,j,k) == 1 %  in C-free
%             U(i,j,k) = 1; I(i,j,k) = 0;   V(i,j,k)=0.1; % dummy value
%         else
%             U(i,j,k) = 0; I(i,j,k) = 1; V(i,j,k)=0;
%         end
%     end
% end
% end
U = U/sum(U(:));  V = V/sum(V(:));  
points = [];

end


function F = gaussian2d(x,y)
global A2d
step = length(A2d);
mu = [x y];
sigma = 0.3;
Sigma = [sigma 0;0 sigma];
x1 = linspace(0,9.2,step); x2 = linspace(0,9.2,step);
[X1,X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(x2),length(x1));
end


function [x,y] = grid2xy(i,j)
global mapLimit B3d A2d
 n = length(A2d);% m = length(B3d{1});
myStart = mapLimit(1); myEnd = mapLimit(2);
step = (myEnd-myStart)/(n-1);
x = myStart+(j-1)*step;
y = myStart+(i-1)*step;

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
recFur = recObs([2  2  4   4;1.5  2.5  2.5  1.5],'furniture');

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
n = 15;  % # of step of sampling
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