%function directional_CT(filename)
clc,clear,close all
filename = 'targInfo.mat';
addpath('Min_August\0807CTarget\infoData')
addpath('Min_August\0807CTarget\classdef')
addpath('helperFuns')
addpath('Rectangle_Decomposition')
addpath('Sort_struct')
addpath('TSP_Matlab')
addpath('Min_August\0807CTarget')
warning off



%%%%%%%% MAIN
global UGV
[~,walls,recFur,cirFur,UGV] = buildAll();


data = load(filename);

targets_raw = data.targInfo{1}';
for i = 1:size(targets_raw)
    targets(i).loc = targets_raw(i,:);
end


recFur(1).vertices = [1.067,0.2794;1.067,0.7366;1.524,0.7366;1.524,0.2794]';
recFur(2).vertices = [1.524,2.54;1.524,2.921;1.981,2.921;1.981,2.54]';
recFur(3).vertices = [1.905,5.461;1.905,6.02;2.413,6.02;2.413,5.461]';
recFur(4).vertices = [4.953,4.826;4.953,5.283;5.41,5.283;5.41,4.826]';
recFur(5).vertices = [4.47,3.277;4.47,3.734;4.928,3.734;4.928,3.277]';
recFur(6).vertices = [0.9906,5.207;0.9906,5.334;2.311,5.334;2.311,5.207]';
recFur(7).vertices = [4.127,5.715;4.127,7.036;4.255,7.036;4.255,5.715]';
recFur(8).vertices = [4.458,3.734;4.458,3.835;6.363,3.835;6.363,3.734]';
recFur(9).vertices = [3.108,0;3.108,1.854;3.191,1.854;3.191,0]';
recFur(10).vertices = [3.124, 3.708;3.124,3.808;3.734,3.728;3.734,3.808]';
recFur(11).vertices = [3.124, 5.132;3.124,5.232;3.734,5.232;3.734,5.132]';
recFur = recFur(1:11);

%modify walls
walls(1).vertices = [0,0,3.988,3.988;
                    -1.499,0,0,-1.499];
walls(2).vertices = [0,0,0.7366,0.7366;
                    0,2.794,2.794,0];
walls(3).vertices = [0,0,0.9398,0.9398;
                    2.794,7.036,7.036,2.794];
walls(4).vertices = [0,0,7.137,7.137;
                     7.036,9.474,9.474,7.036];         
walls(5).vertices = [6.758,6.758,7.137,7.137;
                     4.953,7.036,7.036,4.953];
walls(6).vertices = [6.325,6.325,7.137,7.137;
                     3.378,4.953,4.953,3.378];
                 
walls(7).vertices = [6.629,6.629,7.137,7.137;
                     0.9906,3.378,3.378,0.9906];
                 
walls(8).vertices = [6.325,6.325,7.137,7.137;
                     -0.889,0.9906,0.9906,-0.889];
walls(9).vertices = [3.988,3.988,7.137,7.137;
                     -1.499,-0.889,-0.889,-1.499]; 

x_max = 40;
y_max = 40;


%% resolution CT
L_fov = 0.4;
PHI = 90/180*pi;

resolution_x = 0.1;
resolution_theta = pi/2;

nodes_x = round(x_max/resolution_x);
nodes_theta = round(2*pi/resolution_theta);

CT = zeros(nodes_x,nodes_x,nodes_theta);
CT_TID = {};
index = 1;

theta = pi/2;
v1 = [L_fov*cos(theta-PHI/2),L_fov*sin(theta-PHI/2)];
v2 = [L_fov*cos(theta+PHI/2),L_fov*sin(theta+PHI/2)];
l = 0.0/2;
for w=1:nodes_theta
    theta = resolution_theta*w;
    v1 = [L_fov*cos(theta-PHI/2),L_fov*sin(theta-PHI/2)];
    v2 = [L_fov*cos(theta+PHI/2),L_fov*sin(theta+PHI/2)];
    for i=1:nodes_x
        for j=1:nodes_x
            x_node = i*resolution_x;
            y_node = j*resolution_x;
            CT_flag = 0;
            target_ID = [];
            for k=1:size(targets,2)
                t = targets(k).loc;
                t_v1 = t - v1;
                t_v2 = t - v2;
                X = [t(1),t_v1(1),t_v2(1)];
                Y = [t(2),t_v1(2),t_v2(2)];
                if(inpolygon(x_node+l/2*cos(theta),y_node+l/2*sin(theta),X,Y))
                    CT_flag = 1;
                    target_ID = [target_ID, k];
                else
                    continue;
                end
                
                block_flag = 0;
                %size(walls,2)
                for q = 1:size(walls,2)
                    v = walls(q).vertices;
                    x_b = v(1,:);
                    y_b = v(2,:);
                    if ~isempty(polyxpoly([x_node,t(1)], [y_node,t(2)], x_b, y_b))
                        block_flag = 1;
                        break;
                    end
                    if inpolygon(x_node,y_node,x_b,y_b)
                        block_flag = 1;
                        break;
                    end
                        
                    
                end
                for q=1:size(recFur,2)
                    v = recFur(q).vertices;
                    x_b = v(1,:);
                    y_b = v(2,:);
                    if ~isempty(polyxpoly([x_node,t(1)], [y_node,t(2)], x_b, y_b))
                        block_flag = 1;
                        break;
                    end
                end
            end
            if CT_flag == 1 && block_flag==0
                CT(i,j,w) = 1;
                CT_TID{index} = {x_node,y_node,theta,[target_ID]};
                index = index+1;
            end
            %         save('CT_nodes_5.mat','CT')
            %         save('CT_TID_5.mat','CT_TID')
        end
    end
    
    
    
end
save('CT_nodes_5_small_map3.mat','CT')
save('CT_TID_5_small_map3.mat','CT_TID')
figure
ind_t = 1:size(targets_raw,1);
offset_y= 0;
offset_x = 0;
for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
    plot(xt(1) - offset_x,xt(2)+offset_y,'.r', 'MarkerSize',25)
    text(xt(1) - offset_x,xt(2)-0.1+offset_y,num2str(i))
    hold on
end

for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1)-offset_x;
    y = round(v(2,:),1)+offset_y;
    
    k = convhull(x,y);
    fill(x(k),y(k),'k')
    hold on
end
for i=1:length(walls)
    v = walls(i).vertices;
    x = round(v(1,:),1)-offset_x;
    y = round(v(2,:),1)+offset_y;
    
    k = convhull(x,y);
    fill(x(k),y(k),'k')
    hold on
end
xlim([0,6.5])
ylim([0,8.5])
for i=1:nodes_x
    for j=1:nodes_x
        if (CT(i,j,1)==1)
            plot(i*resolution_x,j*resolution_x,'b*')
            hold on
        end
        
    end
end


%end

%
function varargout = buildAll()
% building targets
load('targetLocs');
targetsLoc = locs';
targets(1,length(targetsLoc)) = points();  % preallocation with default values
for i = 1:length(targetsLoc)
    targets(1,i)=  points([targetsLoc(i,1)   targetsLoc(i,2)],'target');
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
