clear all
close all
clc


% addpath('/Users/min/Google Drive/pathPlanning(1)/infoData')legend 
% addpath('/Users/min/Google Drive/pathPlanning(1)/classdef')
% addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\pathPlanning - Copy\infoData')
% addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\pathPlanning - Copy\classdef')
% addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\pathPlanning - Copy\helperFuns')

addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/infoData')
addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/classdef')
addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/helperFuns')
addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/Rectangle_Decomposition')
addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/Sort_struct')



warning off


ind_t = [12,18,22,21,9,7];

%%%%%%%% MAIN
global UGV
[targets,walls,recFur,cirFur,UGV] = buildAll();
cirFur = [];

targets(12).loc(2) = 2;

recFur(3).vertices = [recFur(3).vertices(1,:)-1;recFur(3).vertices(2,:)-1.5];
    
walls(6).vertices(1,3:4) = [4.7,4.7];
walls(5).vertices(2,2:3) = [4.7,4.7];

x_max = 4.7;
y_max = 4.7;


 targets = [ targets(12),  targets(18),  targets(22),  targets(21),  targets(9),  targets(7)];
 walls = [walls(1), walls(4), walls(5), walls(6)];
 recFur = [recFur(1), recFur(3)];


figure
xlim([0,x_max])
ylim([0,y_max])
for i=1:size(targets,2)
    xt = targets(i).loc;
    plot(xt(1),xt(2),'ko')   
    hold on
end

for i=1:size(walls,2)
    v = walls(i).vertices;
    x = v(1,:);
    y = v(2,:);
    k = convhull(x,y);
    fill(x(k),y(k),[0.5,0.5,0.5])
    hold on
end

for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = v(1,:);
    y = v(2,:);
    k = convhull(x,y);
    fill(x(k),y(k),[0.5,0.5,0.5])
    hold on
end

% 
% %% resolution CT
% tic
% 
% L_fov = 1;
% PHI = 60/180*pi;
% 
% resolution_x = 0.2;
% resolution_theta = pi/2;
% 
% nodes_x = round(x_max/resolution_x);
% nodes_theta = round(2*pi/resolution_theta);
% 
% CT = zeros(nodes_x,nodes_x,nodes_theta);
% CT_TID = {};
% index = 1;
% 
% theta = pi/2;
% v1 = [L_fov*cos(theta-PHI/2),L_fov*sin(theta-PHI/2)];
% v2 = [L_fov*cos(theta+PHI/2),L_fov*sin(theta+PHI/2)];
% 
% 
% % w = 3;
% for w=1:nodes_theta
%     theta = resolution_theta*w;
%     v1 = [L_fov*cos(theta-PHI/2),L_fov*sin(theta-PHI/2)];
%     v2 = [L_fov*cos(theta+PHI/2),L_fov*sin(theta+PHI/2)];
%     w
% for i=1:nodes_x
%     for j=1:nodes_x
%         
%         if i==50 && j==50
%             i
%             j
%         end
%         x_node = i*resolution_x;
%         y_node = j*resolution_x;
%         CT_flag = 0;
%         target_ID = [];
%         for k=1:size(targets,2)
%             t = targets(k).loc;
%             t_v1 = t - v1;
%             t_v2 = t - v2;
%             X = [t(1),t_v1(1),t_v2(1)];
%             Y = [t(2),t_v1(2),t_v2(2)];
%             if(inpolygon(x_node,y_node,X,Y))
%                 CT_flag = 1;
%                 target_ID = [target_ID, k];
%             else
%                 continue;
%             end
%             
%             block_flag = 0;
%             for q=1:size(walls,2)
%                 v = walls(q).vertices;
%                 x_b = v(1,:);
%                 y_b = v(2,:);
%                 if ~isempty(polyxpoly([x_node,t(1)], [y_node,t(2)], x_b, y_b))
%                     block_flag = 1;
%                     break;
%                 end
% 
%             end
%             for q=1:size(recFur,2)
%                 v = recFur(q).vertices;
%                 x_b = v(1,:);
%                 y_b = v(2,:);
%                 if ~isempty(polyxpoly([x_node,t(1)], [y_node,t(2)], x_b, y_b))
%                     block_flag = 1;
%                     break;
%                 end
%             end          
%         end     
%         if CT_flag == 1 && block_flag==0
%             CT(i,j,w) = 1;
%             CT_TID{index} = {x_node,y_node,theta,[target_ID]};
%             index = index+1;
%         end
% %         save('CT_nodes_5.mat','CT')
% %         save('CT_TID_5.mat','CT_TID')
%     end   
% end
% 
% 
% 
% end
% toc
% 
% % save('CT_nodes_5.mat','CT')
% % save('CT_TID_5.mat','CT_TID')
% 
% save('CT_nodes_5_small.mat','CT')
% save('CT_TID_5_small.mat','CT_TID')
% 
% for i=1:nodes_x
%     for j=1:nodes_x
%         if (CT(i,j,1)==1)
%             plot(i*resolution_x,j*resolution_x,'b*')
%             hold on
%         end
%         
%     end
% end
% 





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

