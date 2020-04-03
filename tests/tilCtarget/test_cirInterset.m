
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\infoData')
addpath('C:\Users\Ferrari Lab\Google Drive\pathPlanning\classdef')


global mapLimit   % setting for canvas size 
 mapLimit = [0  9.2];
 
figure;
 [targets,walls,recFur,cirFur,UGV] = buildAll();
 all = {targets,walls,recFur,cirFur,UGV};
 plotAll(all);
 


w = 0.12; h = 0.10;
r = sqrt(w^2+h^2)/2;

% robotCir = cirObs([4.2  2.85],r,'tester');
% robotCir = cirObs([2.65  0.4],r,'tester');
% robotCir = cirObs([2.337  5.181],r,'tester');

robotCir = cirObs([2.51  5.228],r,'tester');

plotObs(robotCir,'r',true);
axis equal
% geiven a location, test if in C free
obstacles = {walls,recFur,cirFur};
boolean = inCFree(UGV,robotCir,obstacles)

 % % NOW TESTING

 % % END TESTING
 
 
 
% function boolean = inCFree(robotCir,obstacles)
% r = robotCir.rad;
% rX = robotCir.loc(1);  rY = robotCir.loc(2);
% hold on
% % plot([robotXs   robotXs(1)],[robotYs   robotYs(1)],'g');
% 
% % check if circle boundary is out of bound
% if rX-r < 0 || rY-r < 0 || rX+r > 9.2 || rY+r > 9.2
%     boolean = false;
%     return
% end
% 
% for groupObs = obstacles  % e.g. groupObs = walls
%     for item = groupObs{1}  % e.g. wall #1
%         if ptInObs(item,rX,rY) % if center is in obstacle; helps on inCircle detection as well, but still need cirXcir test
%             boolean = false;
%             return
%         end
%         if isa(item, 'recObs')  % now check recFur and walls only
%             if polyConflict(robotCir,item.vertices(1,:),item.vertices(2,:)) % if polygon intersect
%                 boolean = false;
%                 return
%             end
%         elseif isa(item,'cirObs')
%               disp('circle obstacle');
%               if cirXcir(robotCir,item) 
%                   boolean = false;
%                   return                  
%               end
%         end
%         
%     end
% end
% boolean = true;
% end
%  
 % generate a C-target distribution in the worldSpace
function A = CFree(obstacles)
global mapLimit
n = 60;  % # of step of sampling
m = 1; % theta steps

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
   
