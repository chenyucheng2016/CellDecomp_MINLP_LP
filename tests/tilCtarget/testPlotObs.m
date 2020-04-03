clear all;
close all;
addpath('C:\Users\Ferrari Lab\Desktop\pathPlanning\classdef')  % add folder of class 

load('walls.mat');
walls(1,length(coors)) = recObs();
for i = 1:length(walls)
    walls(i) = recObs(coors{i},'wall');
end


figure(1)
hold on
xlim([0  9.2])
ylim([0  9.2])
axis equal
plotObjArr(walls,'k'); %plotting walls
alpha(.5)


fill ([0.3667    0.3667    2.1667    2.1667],[ 0.3225    1.1225    1.1225    0.3225],'g');
fill ([5.6218    5.6218    6.7218    6.7218],[0.2526    1.2526    1.2526    0.2526],'g');
fill ([ 2.4012    2.4012    3.2012    3.2012],[4.7000    5.2000    5.2000    4.7000],'g');
fill ([8.2500    8.2500    8.9500    8.9500],[4.7035    5.4035    5.4035    4.7035],'g');
fill ([6.5105    6.5105    7.5105    7.5105],[6.1518    7.9518    7.9518    6.1518],'g');
% chairs
fill ([6.7605    6.7605    7.2605    7.2605],[7.7018    8.2018    8.2018    7.7018],'g');
fill ([6.7605    6.7605    7.2605    7.2605],[5.9018    6.4018    6.4018    5.9018],'g');
fill ([6.2605    6.2605    6.7605    6.7605],[6.8018    7.3018    7.3018    6.8018],'g');
fill ([7.2605    7.2605    7.7605    7.7605],[6.8018    7.3018    7.3018    6.8018],'g');

% rotated
fill ([ 8.0550    7.4463    8.3190    8.9277    8.0550],[0.4425    1.2359    1.9055    1.1121    0.4425],'g');
alpha(.2)


% circles
rectangle('Position',[ 0.3886    1.2696    0.3000    0.3000],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[ 2.2800    0.2835    0.3200    0.3200],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[ 4.0337    2.6860    0.3200    0.3200],'Curvature',[1 1],'FaceColor','k');
rectangle('Position',[ 7.2440    0.4562    0.3000    0.3000],'Curvature',[1 1],'FaceColor','k');
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




