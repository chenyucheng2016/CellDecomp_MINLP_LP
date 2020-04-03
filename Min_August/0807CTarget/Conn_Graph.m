clear
close all
clc

% addpath('/Users/min/Google Drive/pathPlanning(1)/infoData')
% addpath('/Users/min/Google Drive/pathPlanning(1)/classdef')

addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Min_August\0807CTarget\infoData')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Min_August\0807CTarget\classdef')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\helperFuns')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Rectangle_Decomposition')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Sort_struct')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\TSP_Matlab')



% addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy')
% addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/infoData')
% addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/classdef')
% addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/helperFuns')
% addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/Rectangle_Decomposition')
% addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/Sort_struct')
% addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/TSP_Matlab')

w_trans = 0.3;
w_rotat = 1 - w_trans;

cell_max = 4;

warning off

x_max = 5;
y_max = 5;

resolution_x = 0.2;
resolution_theta = pi/2;

%%%%%%%% MAIN
global UGV
[targets,walls,recFur,cirFur,UGV] = buildAll();
% 
% % workspace 2
% targets(3).loc = [2.8, 3];
% targets(5).loc(2) = 2.5;
% targets(9).loc(1) = 4;
% targets(12).loc = [0.5,4];
% targets(15).loc(2) = 1.5;
% targets(21).loc(2) = 0.8;
% 
% recFur(3).vertices = [recFur(3).vertices(1,:)-1;recFur(3).vertices(2,:)-1.5];
% recFur(4).vertices = [recFur(4).vertices(1,:)-5;recFur(4).vertices(2,:)-2];
% recFur(1).vertices = [recFur(1).vertices(1,:)+1;recFur(1).vertices(2,:)+1];

% workspace 3
targets(3).loc = [3.1,3.3];
targets(5).loc = [3.5,1];
targets(9).loc(1) = 4;
targets(12).loc = [0.5,4];
targets(15).loc = [0.3,1.5];
targets(21).loc = [1.2,1.2];
targets(22).loc(1) = 2.6;

recFur(1).vertices = [1.8,1.8,2.6,2.6;0.6,1,1,0.6];
recFur(2).vertices = [0.6,0.6,1,1;1,2.6,2.6,1];
recFur(3).vertices = [1.4,1.4,1.8,1.8;2.2,2.6,2.6,2.2];
recFur(4).vertices = [1.4,1.4,2.2,2.2;1.8,2.2,2.2,1.8];
recFur(5).vertices = [1.8,1.8,2.6,2.6;1.4,1.8,1.8,1.4];

recFur(6).vertices = [2.8,2.8,3.6,3.6;2.2,2.6,2.6,2.2];
recFur(7).vertices = [2.8,2.8,3.6,3.6;3.8,4.2,4.2,3.8];
recFur(8).vertices = [2,2,2.4,2.4;2.8,3.6,3.6,2.8];
recFur(9).vertices = [4,4,4.4,4.4;2.8,3.6,3.6,2.8];

walls(6).vertices(1,3:4) = [4.7,4.7];
walls(5).vertices(2,2:3) = [4.7,4.7];




% ind_t =  [14,29,4,28,6,24,26,1,13,25];
ind_t = [12,18,22,21,9,7,3,5,15];
% targets = targets(ind_t);


load('CT_TID_5_small_map2.mat')

resolution_theta = pi/2;
tot_theta = round(2*pi/resolution_theta);
T_nodes = cell(size(targets,2),tot_theta);


% w=3, theta=pi/2

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
w_xmax = 4.7;
w_ymin = 0.2;
w_ymax = 4.7;

workspace_rect = {[w_xmin,w_xmin,w_xmax,w_xmax;w_ymin,w_ymax,w_ymax,w_ymin]};
% workspace_rect = {[4.7,4.7,9,9; 4.7,9,9,4.7]};
% workspace_rect = {[0.2,0.2,9,9; 0.2,9,9,0.2]};

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

obs_rect = {};
index = 1;
for i=1:size(recFur,2)
    v = recFur(i).vertices;
    obs_rect{index} = v;
    index = index+1;
end
for i=5:size(walls,2)
    v = walls(i).vertices;
    obs_rect{index} = v;
    index = index+1;
end


ind_plot_tar = 4;
decomposed_rect=rect_decomposition(workspace_rect,[tar_rect{:,ind_plot_tar},obs_rect]);
save de_rects_5_2pi_small decomposed_rect

ind_plot_tar = 3;
decomposed_rect=rect_decomposition(workspace_rect,[tar_rect{:,ind_plot_tar},obs_rect]);
save de_rects_5_15pi_small decomposed_rect

ind_plot_tar = 2;
decomposed_rect=rect_decomposition(workspace_rect,[tar_rect{:,ind_plot_tar},obs_rect]);
save de_rects_5_pi_small decomposed_rect

ind_plot_tar = 1;
decomposed_rect=rect_decomposition(workspace_rect,[tar_rect{:,ind_plot_tar},obs_rect]);
save de_rects_5_halfpi_small decomposed_rect
% save de_rects_2pi decomposed_rect 

%% 
ind_plot_tar = 4;
load('de_rects_5_2pi_small.mat')



figure

for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
    if any(xt > 4.7)
        continue
    end
    
%     plot(xt(1),xt(2),'ko') 
    plot(xt(1),xt(2),'.r', 'MarkerSize',30)
    text(xt(1),xt(2)-0.1,num2str(i))
    hold on
end

for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
    if any(x>4.7) || any(y>4.7)
        continue
    end
    
    k = convhull(x,y);
    fill(x(k),y(k),'k')
%     text(x,y-0.1,num2str(i))
    hold on
end



% plot the rectangles
for i=1:size(decomposed_rect,2)
    v = decomposed_rect(i);
    v = v{1};
    plot(v(1,:),v(2,:),'k-')
    plot([v(1,1),v(1,4)],[v(2,1),v(2,4)],'k-')
%     text(v(1,1)+0.1,v(2,1)+0.1,num2str(i));
    
end
for i=1:size(tar_rect)
    v = tar_rect(i, ind_plot_tar);
    v = v{1};
    if isempty(v)
        continue;
    end
    if any(v> 4.5)
        continue
    end
    
    fill(v(1,:),v(2,:),'b')
%     text(v(1,1)+0.1,v(2,1)+0.1,num2str(i));
end

for i=1:size(walls,2)
    v = walls(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
    if i==1
        y(2:3) = 4.7;
    end
    if i==4
        x(3:4) = 4.7;
    end
    
    if any(x>4.9) || any(y>4.9)
        continue
    end
    
    k = convhull(x,y);
    fill(x(k),y(k),'k')
%     text(x,y-0.1,num2str(i))
    hold on
end


axis equal
set(gca,'XTick',[0 : 1: x_max]);
set(gca,'YTick',[0 : 1: y_max]);
xlim([0,x_max])
ylim([0,y_max])

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


%% plot the smaller cells
figure
axis equal
ind_plot_tar = 3;

for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
    if any(xt > 4.7)
        continue
    end
    
    plot(xt(1),xt(2),'ko')  
    text(xt(1),xt(2)-0.1,num2str(i))
    hold on
end


for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
    if any(x>4.7) || any(y>4.7)
        continue
    end
    
    k = convhull(x,y);
    fill(x(k),y(k),[0.5,0.5,0.5])
%     text(x,y-0.1,num2str(i))
    hold on
end



% plot the rectangles
for i=ID_2pi : ID_15pi
    v = Nodes_TSP(i).vertices{1};
    v = v';
    plot(v(1,:),v(2,:),'k-')
    plot([v(1,1),v(1,4)],[v(2,1),v(2,4)],'k-')
%     text(v(1,1)+0.1,v(2,1)+0.1,num2str(i));
    
end
for i=1:size(tar_rect)
    v = tar_rect(i, ind_plot_tar);
    v = v{1};
    if isempty(v)
        continue;
    end
    if any(v>4.7)
        continue
    end
    
    fill(v(1,:),v(2,:),'b')
    
end



for i=1:size(walls,2)
    v = walls(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
    if i==1
        y(2:3) = 4.7;
    end
    if i==4
        x(3:4) = 4.7;
    end
    
    if any(x>4.9) || any(y>4.9)
        continue
    end
    
    k = convhull(x,y);
    fill(x(k),y(k),[0.5,0.5,0.5])
%     text(x,y-0.1,num2str(i))
    hold on
end
 


title(['Result of Approximate Cell Decomposition with c\_max=',num2str(cell_max),', C-Targets at \theta=2\pi'])
set(gca,'XTick',[0 : 1: x_max]);
set(gca,'YTick',[0 : 1: y_max]);
xlim([0,x_max])
ylim([0,y_max])
axis equal


%% construct the connectivity graph and add targets to nodes
fprintf('connectivity begins\n');

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

l = 0.2/2;
h = 0.1/2;

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



%% Solve TSP
fprintf('TSP construction\n');

nStops = length(Nodes_TSP) + 1;

idxs_init = nchoosek(1:nStops,2);
idxs_init = [idxs_init;[idxs_init(:,2),idxs_init(:,1)]];
distance_TSP = zeros(size(keySet,2)*2 + length(Nodes_TSP)*2,1);
idxs = zeros(length(distance_TSP),2);

not_conn = [];

index = 1;

for i = 1:size(idxs_init,1)
    n1 = idxs_init(i,1);
    n2 = idxs_init(i,2);
    if n1<n2
        st = [num2str(n1),',',num2str(n2)];
    else
        st = [num2str(n2),',',num2str(n1)];
    end
    k = {st};
    if n1==nStops || n2==nStops
        % dummy
        distance_TSP(index) = 0;
        idxs(index,:) = [n1, n2];
        index = index + 1;
        continue
    end
    if isKey(conn,k)

        v1 = Nodes_TSP(n1).vertices;
        v2 = Nodes_TSP(n2).vertices;
        flag = false;

        R1 = v1(1);
        R1 = R1{1}';
        R2 = v2(1);
        R2 = R2{1}';
        if rect_adj(R1, R2)==1
            if abs(Nodes_TSP(n1).theta - Nodes_TSP(n2).theta) < 1e-6
                % Node i and Node j are on the same theta plane
                if abs(Nodes_TSP(n1).theta - pi/2) < 1e-6
                    if min(R2(2,:)) >= max(R1(2,:))
                        flag = true;
                    end
                else 
                    if abs(Nodes_TSP(n1).theta - 3*pi/2) <1e-6
                        if min(R1(2,:)) >= max(R2(2,:))
                            flag = true;
                        end
                    else
                        if abs(Nodes_TSP(n1).theta - pi) <1e-6
                            if min(R1(1,:)) >= max(R2(1,:))
                                flag = true;
                            end
                        else
                            % theta = 2pi
                            if min(R2(1,:)) >= max(R1(1,:))
                                flag = true;
                            end
                        end
                    end
                end
            else
                flag = true;
            end
        end
        
        if flag
            distance_TSP(index) = conn(st);
            idxs(index,:) = [n1, n2];
            index = index + 1;
        end
        
       
    end
end



%% examine conn graph
ind1 = size(de_rects_2pi,2);
ind2 = ind1 + size(de_rects_15pi,2);
ind3 = ind2 + size(de_rects_pi,2);
ind4 = ind3 + size(de_rects_05pi,2);



s = zeros(1, size(conn_pair,1));
t = zeros(1, size(conn_pair,1));
weights = zeros(1, size(conn_pair,1));
x = zeros(1, size(Nodes_TSP,2));
y = zeros(1, size(Nodes_TSP,2));
z = zeros(1, size(Nodes_TSP,2));

begin = 0;
final = ind1;


for i = 1:size(conn_pair,1)
    n1 = conn_pair(i, 1);
    n2 = conn_pair(i, 2);
    
    s(i) = conn_pair(i, 1);
    t(i) = conn_pair(i, 2);
    weights(i) = distance_TSP(i);
end


for i=1:size(Nodes_TSP,2)
    s(size(conn_pair,1)+i) = size(Nodes_TSP,2)+1;
    t(size(conn_pair,1)+i) = Nodes_TSP(i).ID;
    weights(size(conn_pair,1)+i) = 0;
    
end

for i = 1:size(Nodes_TSP,2)
    x(i) = mean(Nodes_TSP(i).vertices{1}(:,1));
    y(i) = mean(Nodes_TSP(i).vertices{1}(:,2));
    z(i) = Nodes_TSP(i).theta;
end
x(length(x)+1) = 0;
y(length(y)+1) = 0;
z(length(z)+1) = 0;

G = graph(s,t,weights);

tot = 0;
for i = 1:length(black_cells)
    stops = black_cells{i};
    tot = tot + length(stops);
end


figure
p = plot(G,'XData',x,'YData',y,'ZData',z);%,'EdgeLabel',G.Edges.Weight);
p.EdgeColor = [0.9,0.9,0.9];
highlight(p, [begin+1:final],'NodeColor','b','MarkerSize',6)
highlight(p, [size(Nodes_TSP,2)-tot+1:size(Nodes_TSP,2)],'NodeColor','r','MarkerSize',6)



xlabel('X(m)')
ylabel('Y(m)')
zlabel('\theta(rad)')
title('Connectivity Graph in Configuration Space')
text(-4,-3,'Red: C-Targets, Green: Start, Magenta:dummy')








%% helper functions
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

