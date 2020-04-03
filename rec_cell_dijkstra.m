%start = tic;
%function [tot_cost,tot_Info,totClass,totMeasureCost] = rec_cell_dijkstra(filename)
clc,clear,close all
start_tic = (tic);
%filename = 't
filename = 'targInfo.mat';
addpath(genpath('Targets/'));
addpath('Min_August\0807CTarget\infoData')
addpath('Min_August\0807CTarget\classdef')
addpath('helperFuns')
addpath('Rectangle_Decomposition')
addpath('Sort_struct')
addpath('TSP_Matlab')
addpath('Min_August\0807CTarget')

w_trans = 0.99;
w_rotat = 1 - w_trans;

cell_max = 5;

warning off

x_max = 40.2;
y_max = 40.2;

resolution_x = 0.1;

%%%%%%%% MAIN
global UGV
[~,walls,recFur,cirFur,UGV] = buildAll();

data = load(filename);
targets_raw = data.targInfo{1}';
data.targInfo{2} = {'Object','Sphere','Computer','Object','Box','BrownBox','Object','Object','Watermelon','Sphere','Computer'};
data.targInfo{3} = [3,1,5,4,5,3,2,1,7,7,5];
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
ind_t = 1:11;%targets to visit

obs_VR = [8         -.25    12       .5;
          -20       -.25    12       .5;
          -.25      8       .5       12;
          -.25      -20     .5       12;
          0.86	    18.13	1.54    1.53;
          -19.78	-9.53	1.21	1.21;
          -19.61	10.39	2.23	2.01;
          -10.85	-15.07	1.14	1.72;
          -19.80	-17.41	0.56	0.56;
          -0.80	-15.36	0.51	1.79;
          15.89	0.58	3.16	2.45;
          -19.74	-14.75	0.89	2.11;
          9.51	-19.79	1.93	0.87;
          -19.50	9.10	0.81	0.89;
          ];
numObs = size(obs_VR,1);
obs_verts = {};
for i = 1:numObs
    x0 = obs_VR(i,1);y0 = obs_VR(i,2);L = obs_VR(i,3);H = obs_VR(i,4);
    obs_verts{end+1} = [x0,x0+L,x0+L,x0;y0,y0,y0+H,y0+H]';
end
load('CT_TID_5_small_map3.mat')
%1,2: x, y; theta:3 TID: 4 

resolution_theta = pi/2;
tot_theta = round(2*pi/resolution_theta);
T_nodes = cell(size(targets,2),tot_theta);

for i=1:size(CT_TID,2)
    x = round(CT_TID{i}{1},1);
    y = round(CT_TID{i}{2},1);
    theta = CT_TID{i}{3};
    TID = CT_TID{i}{4};
    ind_theta = round(theta/resolution_theta);
    T_coord = T_nodes{TID, ind_theta};
    T_coord = [T_coord;x,y];
    T_nodes{TID(1), ind_theta} = T_coord;   
end

%generate C-target(yc)
Rectangles_T = cell(size(targets,2),tot_theta);
for i = 1:size(Rectangles_T,1)
    for k=1:size(T_nodes,2)
        rects = {};
        index = 1;
       
        nodes = T_nodes{i, k};
        if isempty(nodes)
            continue    
        end
        nodes = sortrows(nodes, 2);
        y_min = nodes(1,2);
        y_max_val = nodes(size(nodes, 1),2);
        y_nodes = [y_min:resolution_x:y_max_val];
        min_max_data = zeros(length(y_nodes),3);
        for j = 1:length(y_nodes)
            yj = y_nodes(j);
            yj = round(yj,1);
            x_nodes = nodes(nodes(:,2)==yj);
            min_max_data(j,:) = [min(x_nodes), max(x_nodes), yj];
        end
        for j = 1:size(min_max_data,1)-1
            xleft = min(min_max_data(j:j+1,1));
            xright = min(min_max_data(j:j+1,2));
            if xleft==xright
                continue;
            end
            ydown = min_max_data(j,3);
            yup = min_max_data(j+1,3);
            rect = [xleft,ydown; xleft,yup; xright,yup; xright,ydown];
            rects{index} = rect;
            index = index+1;
        end
        Rectangles_T{i, k} = rects;
    end
end



w_xmin = 0.2;
w_xmax = 40.2;
w_ymin = 0.2;
w_ymax = 40.2;

workspace_rect = {[w_xmin,w_xmin,w_xmax,w_xmax;w_ymin,w_ymax,w_ymax,w_ymin]};
tar_rect = cell(1000,tot_theta);   % sufficiently large number
for k=1:tot_theta
    index = 1;
    for i=1:size(Rectangles_T,1)
        for j=1:size(Rectangles_T{i, k},2)
            rectangle = Rectangles_T{i, k}{j}';
            if ~isempty(rectangle)
                tar_rect{index, k} = rectangle;
                index = index+1;
                %text(rectangle(1,1),rectangle(2,1),num2str(i))
            end

        end
    end
end
obs_rect_13 = {};
index = 1;
for i=1:size(recFur,2)
    v = recFur(i).vertices;
    v(1,1:2) = v(1,1:2) - 0.1;
    v(1,3:4) = v(1,3:4) + 0.1;
    v(2,2:3) = v(2,2:3) + 0.2;
    v(2,1) = v(2,1) - 0.2;
    v(2,4) = v(2,4) - 0.2;
    obs_rect_13{index} = v;
    index = index+1;
end
for i=1:size(walls,2)
    v = walls(i).vertices;
    v(1,1:2) = v(1,1:2) - 0.1;
    v(1,3:4) = v(1,3:4) + 0.1;
    v(2,2:3) = v(2,2:3) + 0.2;
    v(2,1) = v(2,1) - 0.2;
    v(2,4) = v(2,4) - 0.2;
    obs_rect_13{index} = v;
    index = index+1;
end


obs_rect_24 = {};
index = 1;
for i=1:size(recFur,2)
    v = recFur(i).vertices;
    v(1,1:2) = v(1,1:2) - 0.2;
    v(1,3:4) = v(1,3:4) + 0.2;
    v(2,2:3) = v(2,2:3) + 0.1;
    v(2,1) = v(2,1) - 0.1;
    v(2,4) = v(2,4) - 0.1;
    obs_rect_24{index} = v;
    index = index+1;
end
for i=1:size(walls,2)
    v = walls(i).vertices;
    v(1,1:2) = v(1,1:2) - 0.2;
    v(1,3:4) = v(1,3:4) + 0.2;
    v(2,2:3) = v(2,2:3) + 0.1;
    v(2,1) = v(2,1) - 0.1;
    v(2,4) = v(2,4) - 0.1;
    obs_rect_24{index} = v;
    index = index+1;
end



ind_plot_tar = 4;
decomposed_rect=rect_decomposition(workspace_rect,[tar_rect{:,ind_plot_tar},obs_rect_24]);%@ /Rectangle_decomposition
save de_rects_5_2pi_small decomposed_rect

ind_plot_tar = 3;
decomposed_rect=rect_decomposition(workspace_rect,[tar_rect{:,ind_plot_tar},obs_rect_13]);
save de_rects_5_15pi_small decomposed_rect

ind_plot_tar = 2;
decomposed_rect=rect_decomposition(workspace_rect,[tar_rect{:,ind_plot_tar},obs_rect_24]);
save de_rects_5_pi_small decomposed_rect

ind_plot_tar = 1;
decomposed_rect=rect_decomposition(workspace_rect,[tar_rect{:,ind_plot_tar},obs_rect_13]);
save de_rects_5_halfpi_small decomposed_rect

%% 
ind_plot_tar = 4;
load('de_rects_5_2pi_small.mat')

%% construct the nodes in the connectivity graph for TSP

load('de_rects_5_2pi_small.mat')
de_rects_2pi = decomposed_rect;
load('de_rects_5_15pi_small.mat')
de_rects_15pi = decomposed_rect;
load('de_rects_5_pi_small.mat')
de_rects_pi = decomposed_rect;
load('de_rects_5_halfpi_small.mat')
de_rects_05pi = decomposed_rect;

tot_de = size(de_rects_2pi,2) + size(de_rects_15pi,2) + size(de_rects_pi,2) + size(de_rects_05pi,2);
tot = tot_de;
Nodes_TSP (tot).ID = -1;
Nodes_TSP (tot).vertices = {};
Nodes_TSP (tot).TID = -1;   % -1 for not target
Nodes_TSP (tot).theta = -1;
Nodes_TSP (tot).min_x = 10;


ID = 1;
% 2 pi data
for i=1:size(de_rects_2pi,2)
%     Nodes_TSP(ID).ID = ID;
    xy = de_rects_2pi(i);
    xy = xy{1}';
    if min(xy(1,:))<w_xmin || max(xy(1,:))>w_xmax || min(xy(2,:))<w_ymin || max(xy(2,:)) > w_ymax
        continue
    end
    % divide large cells
    c_x_min = min(xy(:,1));
    c_x_max = max(xy(:,1));
    c_y_min = min(xy(:,2));
    c_y_max = max(xy(:,2));
    num_x = ceil((c_x_max - c_x_min)/cell_max);
    num_y = ceil((c_y_max - c_y_min)/cell_max);
    dx = (c_x_max - c_x_min)/num_x;
    dy = (c_y_max - c_y_min)/num_y;
    for m = 1:num_x
        for n = 1:num_y
            x_bl = round(c_x_min + dx*(m-1),1);
            y_bl = round(c_y_min + dy*(n-1),1);
            x_ur = round(c_x_min + dx*m,1);
            y_ur = round(c_y_min + dy*n,1);
            v = [x_bl,y_bl;x_ur,y_bl;x_ur,y_ur;x_bl,y_ur];
            
            Nodes_TSP(ID).ID = ID;
            Nodes_TSP(ID).vertices = {v}; 
            Nodes_TSP(ID).theta = 2*pi;
            Nodes_TSP(ID).TID = -1;
            Nodes_TSP (ID).min_x = min(v(:,1));
            ID = ID + 1;
        end
    end
end

ID_2pi = ID - 1; % store the ID interval for 2 pi

% 3/2pi data
for i=1:size(de_rects_15pi,2)
%     Nodes_TSP(ID).ID = ID;
    xy = de_rects_15pi(i);
    xy = xy{1}';
    % divide large cells
    c_x_min = min(xy(:,1));
    c_x_max = max(xy(:,1));
    c_y_min = min(xy(:,2));
    c_y_max = max(xy(:,2));
    num_x = ceil((c_x_max - c_x_min)/cell_max);
    num_y = ceil((c_y_max - c_y_min)/cell_max);
    dx = (c_x_max - c_x_min)/num_x;
    dy = (c_y_max - c_y_min)/num_y;
    for m = 1:num_x
        for n = 1:num_y
            x_bl = round(c_x_min + dx*(m-1),1);
            y_bl = round(c_y_min + dy*(n-1),1);
            x_ur = round(c_x_min + dx*m,1);
            y_ur = round(c_y_min + dy*n,1);
            v = [x_bl,y_bl;x_ur,y_bl;x_ur,y_ur;x_bl,y_ur];
            
            Nodes_TSP(ID).ID = ID;
            Nodes_TSP(ID).vertices = {v}; 
            Nodes_TSP(ID).theta = 3/2*pi;
            Nodes_TSP(ID).TID = -1;
            Nodes_TSP (ID).min_x = min(v(:,1));
            ID = ID + 1;
        end
    end
end


ID_15pi = ID - 1;

% pi
for i=1:size(de_rects_pi,2)
%     Nodes_TSP(ID).ID = ID;
    xy = de_rects_pi(i);
    xy = xy{1}';
    
    % divide large cells
    c_x_min = min(xy(:,1));
    c_x_max = max(xy(:,1));
    c_y_min = min(xy(:,2));
    c_y_max = max(xy(:,2));
    num_x = ceil((c_x_max - c_x_min)/cell_max);
    num_y = ceil((c_y_max - c_y_min)/cell_max);
    dx = (c_x_max - c_x_min)/num_x;
    dy = (c_y_max - c_y_min)/num_y;
    for m = 1:num_x
        for n = 1:num_y
            x_bl = round(c_x_min + dx*(m-1),1);
            y_bl = round(c_y_min + dy*(n-1),1);
            x_ur = round(c_x_min + dx*m,1);
            y_ur = round(c_y_min + dy*n,1);
            v = [x_bl,y_bl;x_ur,y_bl;x_ur,y_ur;x_bl,y_ur];
            
            Nodes_TSP(ID).ID = ID;
            Nodes_TSP(ID).vertices = {v}; 
            Nodes_TSP(ID).theta = pi;
            Nodes_TSP(ID).TID = -1;
            Nodes_TSP (ID).min_x = min(v(:,1));
            ID = ID + 1;
        end
    end
end

ID_pi = ID - 1;

% pi/2
for i=1:size(de_rects_05pi,2)
%     Nodes_TSP(ID).ID = ID;
    xy = de_rects_05pi(i);
    xy = xy{1}';
    
    % divide large cells
    c_x_min = min(xy(:,1));
    c_x_max = max(xy(:,1));
    c_y_min = min(xy(:,2));
    c_y_max = max(xy(:,2));
    num_x = ceil((c_x_max - c_x_min)/cell_max);
    num_y = ceil((c_y_max - c_y_min)/cell_max);
    dx = (c_x_max - c_x_min)/num_x;
    dy = (c_y_max - c_y_min)/num_y;
    for m = 1:num_x
        for n = 1:num_y
            x_bl = round(c_x_min + dx*(m-1),1);
            y_bl = round(c_y_min + dy*(n-1),1);
            x_ur = round(c_x_min + dx*m,1);
            y_ur = round(c_y_min + dy*n,1);
            v = [x_bl,y_bl;x_ur,y_bl;x_ur,y_ur;x_bl,y_ur];
            
            Nodes_TSP(ID).ID = ID;
            Nodes_TSP(ID).vertices = {v}; 
            Nodes_TSP(ID).theta = pi/2;
            Nodes_TSP(ID).TID = -1;
            Nodes_TSP(ID).min_x = min(v(:,1));
            ID = ID + 1;
        end
    end
end

ID_05pi = ID - 1;

conn_pair = [];

%R1(:,i)=[x;y], the ith vertex of R1

for i = 1:length(Nodes_TSP)
    for j = i+1:length(Nodes_TSP)
        if i==53 && j==54
            temp = 1;
        end
        
        ID_1 = Nodes_TSP(i).ID;
        ID_2 = Nodes_TSP(j).ID;
        if abs(Nodes_TSP(i).theta - Nodes_TSP(j).theta) > pi/2
            continue
        end
        v1 = Nodes_TSP(i).vertices;
        v2 = Nodes_TSP(j).vertices;
        flag = false;

        R1 = v1(1);
        R1 = R1{1}';
        R2 = v2(1);
        R2 = R2{1}';
        if rect_adj(R1, R2)==1
            if abs(Nodes_TSP(i).theta - Nodes_TSP(j).theta) < 1e-6
                % Node i and Node j are on the same theta plane
                if abs(Nodes_TSP(i).theta - pi/2) < 1e-6 || abs(Nodes_TSP(i).theta - 3*pi/2) <1e-6
                    % theta = pi/2 or 3*pi/2
                    if min(R1(2,:)) >= max(R2(2,:)) || min(R2(2,:)) >= max(R1(2,:))
                        flag = true;
                    end
                else
                    % theta = pi or 2*pi
                    if min(R1(1,:)) >= max(R2(1,:)) || min(R2(1,:)) >= max(R1(1,:))
                        flag = true;
                    end
                end
            else
                flag = true;
            end
        end
        if flag
            conn_pair = [conn_pair;ID_1, ID_2];
        end

    end
end


L0 = length(Nodes_TSP);
black_cells = cell(length(ind_t),1);

l = 0.0/2;
h = 0.0/2;

% add targets
for idt=1:length(ind_t)
    i = ind_t(idt);
    TSP_nodes_Ti = [];
    for k = 1:4
        t_theta = pi/2*k;
        t = Rectangles_T(i, k);
        tt = t{1};
        if isempty(tt)
            continue
        end
        for m=1:size(tt, 2)
            ttt = tt(m);
            xy = ttt{1}';
            
            x_r = mean(xy(1,:));
            y_r = mean(xy(2,:));
            x = [x_r+l*cos(t_theta)+h*sin(t_theta),x_r+l*cos(t_theta)-h*sin(t_theta),...
            x_r-l*cos(t_theta)-h*sin(t_theta),x_r-l*cos(t_theta)+h*sin(t_theta)];
            y = [y_r+l*sin(t_theta)-h*cos(t_theta),y_r+l*sin(t_theta)+h*cos(t_theta),...
            y_r-l*sin(t_theta)+h*cos(t_theta),y_r-l*sin(t_theta)-h*cos(t_theta)];
            
            R2 = [x;y];
            % check if CT overlaps with CB
            is_CB = false;
            for nCB = 1:length(recFur)
                v = recFur(nCB).vertices;
                [out, area] =rect_relation(R1, R2);
                if out ~= 1
                    is_CB = true;
                end
            end
            if is_CB
                continue
            end
            
            Nodes_TSP(ID).ID = ID;
            t_xy = mean(ttt{1},1);
            Nodes_TSP(ID).vertices = {xy'}; 
            Nodes_TSP(ID).theta = t_theta;
            Nodes_TSP(ID).TID = i;
            Nodes_TSP (ID).min_x = t_xy(1);
            TSP_nodes_Ti = [TSP_nodes_Ti, ID];
            ID = ID + 1;
        end
    end
    black_cells{idt} = TSP_nodes_Ti;
end

% check connectivity
for i = L0+1 : length(Nodes_TSP)
    t_theta = Nodes_TSP(i).theta;
    v2 = Nodes_TSP(i).vertices;
    R2 = v2(1);
    R2 = R2{1}';
    for j = 1:L0
        is_conn = false;
        v1 = Nodes_TSP(j).vertices;
        R1 = v1(1);
        R1 = R1{1}';
        if abs(Nodes_TSP(j).theta - t_theta) > pi/2
            continue
        end
        if rect_adj(R1, R2)
            if abs(t_theta - Nodes_TSP(j).theta) < 1e-6
                % Node i and Node j are on the same theta plane
                if abs(t_theta - pi/2) < 1e-6 || abs(t_theta - 3*pi/2) <1e-6
                    % theta = pi/2 or 3*pi/2
                    if min(R1(2,:)) >= max(R2(2,:)) || min(R2(2,:)) >= max(R1(2,:))
                        is_conn = true;
                    end
                else
                    % theta = pi or 2*pi
                    if min(R1(1,:)) >= max(R2(1,:)) || min(R2(1,:)) >= max(R1(1,:))
                        is_conn = true;
                    end
                end
            else
                is_conn = true;
            end 
        end
        if is_conn
            conn_pair = [conn_pair;Nodes_TSP(j).ID, Nodes_TSP(i).ID];
        end

    end
end

%% construct the distance
keySet = {};
valueSet = [];
for i = 1:size(conn_pair,1)
    n1 = conn_pair(i,1);
    n2 = conn_pair(i,2);
    st = [num2str(n1), ',',num2str(n2)];
    x1 = mean(Nodes_TSP(n1).vertices{1}(:,1));
    y1 = mean(Nodes_TSP(n1).vertices{1}(:,2));
    theta1 = Nodes_TSP(n1).theta;
    x2 = mean(Nodes_TSP(n2).vertices{1}(:,1));
    y2 = mean(Nodes_TSP(n2).vertices{1}(:,2));
    theta2 = Nodes_TSP(n2).theta;
    keySet = [keySet,st];
    valueSet = [valueSet,sqrt(w_trans*(x1-x2)^2 + w_trans*(y1-y2)^2 + w_rotat*(theta1-theta2)^2)];
end
conn = containers.Map(keySet,valueSet);
%construct adjacency matrix and cost map
numNodes = length(Nodes_TSP);
Adjacency = zeros(numNodes);
CostMap = 9e9*ones(numNodes);
for i = 1:size(conn_pair,1)
    n1 = conn_pair(i,1);
    n2 = conn_pair(i,2);
    Adjacency(n1,n2) = 1;
    Adjacency(n2,n1) = 1;
    CostMap(n1,n2) = valueSet(i);
    CostMap(n2,n1) = valueSet(i);
end
targetCostMap = zeros(11);
[~,numNodes] = size(Nodes_TSP);
disMin = 9e9;
startNode = -1;
for i = 1:numNodes
    vert = Nodes_TSP(i).vertices;
    vert = vert{1};
    x_center = (max(vert(:,1)) + min(vert(:,1)))/2;
    y_center = (max(vert(:,2)) + min(vert(:,2)))/2;
    if norm([x_center,y_center] - [4.5,0]) < disMin
        disMin = norm([x_center,y_center] - [5,0]);
        startNode = Nodes_TSP(i).ID;
    end
end
cites = startNode;
for i = 1:11
    cites = [cites,black_cells{i}(1)];
end
paths_btw_targets = cell(31);
for i = 1:12
    SID = cites(i);
    for j = 1:12
        FID = cites(j);
        [path_cost,path_hist] = dijkstra(Adjacency,CostMap,SID,FID);
        targetCostMap(i,j) = path_cost;
        path_btw_targets{i,j} = path_hist;
    end
end
%%%%%%%%%%%%%%%
%the weight of three objectives

omegas = [0.0001,7.5,0.3];
init_features = data.targInfo{2};
initFeatureCode = zeros(1,11);
initFeatureLevel = zeros(1,11);
for i = 1:11
    initFeatureCode(i) = encodeFeatures(init_features(i));
    initFeatureLevel(i) = computeFeatureLevel(initFeatureCode(i));
end
maxMeasures = 3 - initFeatureLevel;
moneyUpper = 12;
X = optimalPath(targetCostMap, maxMeasures, omegas, moneyUpper, initFeatureCode);


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
function totClass  = computeClassPerformance(tar_Measured,targetInfos,unvisitedTargets)
init_features = targetInfos{2};
identities = targetInfos{3};
init_features_code = zeros(1,30);
for i = 1:11
    init_features_code(i) = encodeFeatures(init_features(i));
end
totClass = 0;
for i = 1:11
    if ~ismember(i,unvisitedTargets)
        feature = init_features_code(i);
        level = computeFeatureLevel(feature);
        if level == 3 || tar_Measured(i) == 0
            totClass = makeDecisions(feature,totClass);
        elseif level == 2
            totClass = makeDecisions(identities(i),totClass);
        elseif level == 1
            if tar_Measured(i) == 2
                totClass = makeDecisions(identities(i),totClass);
            elseif tar_Measured(i) == 1
                if ismember(identities(i),[3,4])
                    final_feature = 10;
                elseif ismember(identities(i),[5,6])
                    final_feature = 11;
                elseif ismember(identities(i),[1,2])
                    final_feature = 9;
                elseif ismember(identities(i),[7,8])
                    final_feature = 12;
                end
                totClass = makeDecisions(final_feature,totClass);
            end
        elseif level == 0
            if tar_Measured(i) == 3
                totClass = makeDecisions(identities(i),totClass);
            elseif tar_Measured(i) == 2
                if ismember(identities(i),[3,4])
                    final_feature = 10;
                elseif ismember(identities(i),[5,6])
                    final_feature = 11;
                elseif ismember(identities(i),[1,2])
                    final_feature = 9;
                elseif ismember(identities(i),[7,8])
                    final_feature = 12;
                end
                totClass = makeDecisions(final_feature,totClass);
            elseif tar_Measured(i) == 1
                if ismember(identities(i),[3,4,5,6])
                    final_feature = 13;
                elseif ismember(identities(i),[1,2,7,8])
                    final_feature = 14;
                end
                totClass = makeDecisions(final_feature,totClass);
            end
        end
    end
end
end


function totClass = makeDecisions(cur_iden,totClass)
if cur_iden == 1
    if rand() <= 0.93
        totClass = totClass + 1;
    end
elseif cur_iden == 2
    if rand() <= 0.59
        totClass = totClass + 1;
    end
elseif cur_iden == 3
    if rand() <= 0.88
        totClass = totClass + 1;
    end
elseif cur_iden == 4
    if rand() <= 0.95
        totClass = totClass + 1;
    end
elseif cur_iden == 5
    if rand() <= 0.69
        totClass = totClass + 1;
    end
elseif cur_iden == 6
    if rand() > 0.39
        totClass = totClass + 1;
    end
elseif cur_iden == 7
    if rand() > 0.14
        totClass = totClass + 1;
    end
elseif cur_iden == 8
    if rand() <= 0.94
        totClass = totClass + 1;
    end
elseif cur_iden == 9
    if rand() <= 0.65
        totClass = totClass + 1;
    end
elseif cur_iden == 10
    if rand() <= 0.58
        totClass = totClass + 1;
    end
elseif cur_iden == 11
    if rand() <= 0.59
        totClass = totClass + 1;
    end
elseif cur_iden == 12
    if rand() <= 0.74
        totClass = totClass + 1;
    end
elseif cur_iden == 13
    if rand() <= 0.54
        totClass = totClass + 1;
    end
elseif cur_iden == 14
    if rand() <= 0.53
        totClass = totClass + 1;
    end
elseif cur_iden == 15
    if rand() <= 0.51
        totClass = totClass + 1;
    end
end
end
