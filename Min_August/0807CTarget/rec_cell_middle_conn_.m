clear 
close all
clc

addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Min_August\0807CTarget\infoData')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Min_August\0807CTarget\classdef')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\helperFuns')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Rectangle_Decomposition')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Sort_struct')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\TSP_Matlab')


w_trans = 0.3;
w_rotat = 1 - w_trans;

cell_max = 1;

warning off

Room = 3;

w_xmin = 3;
w_xmax = 6;
w_ymin = 3;
w_ymax = 6;


t3 = 110; 
t1 = 65; s1 = 98;
t2 = 75; s2 = 111;
t4 = 141; s4 = 63;









x_max = w_xmax;
y_max = w_ymax;

resolution_x = 0.2;
resolution_theta = pi/2;

%%%%%%%% MAIN
global UGV
[targets,walls,recFur,cirFur,UGV] = buildAll();
obs_all = [recFur,walls];




ind_t = [];
for i = 1:length(targets)
    v = targets(i).loc;
    if v(1) <= w_xmax && v(1) >=w_xmin && v(2) <=w_ymax && v(2) >=w_ymin
        ind_t = [ind_t,i];
    end
end


% ind_t =  [14,29,4,28,6,24,26,1,13,25];
% targets = targets(ind_t);


load('CT_TID_5_map4.mat')

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
for m = 1:length(ind_t)
    i = ind_t(m);
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




workspace_rect = {[w_xmin,w_xmin,w_xmax,w_xmax;w_ymin,w_ymax,w_ymax,w_ymin]};
% workspace_rect = {[4.7,4.7,9,9; 4.7,9,9,4.7]};
% workspace_rect = {[0.2,0.2,9,9; 0.2,9,9,0.2]};

tar_rect = cell(1,tot_theta);   % sufficiently large number


for k=1:tot_theta
    index = 1;
    for m=1:length(ind_t)
        i = ind_t(m);
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
decomposed_rect=rect_decomposition(workspace_rect,[tar_rect{:,ind_plot_tar},obs_rect_24]);
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
% save de_rects_2pi decomposed_rect 


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
    if min(xy(:,1))<w_xmin || max(xy(:,1))>w_xmax || min(xy(:,2))<w_ymin || max(xy(:,2)) > w_ymax
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
    
    
    
%     Nodes_TSP(ID).vertices = {xy}; 
%     Nodes_TSP(ID).theta = 2*pi;
%     Nodes_TSP(ID).TID = -1;
%     Nodes_TSP(ID).min_x = min(xy(:,1));
%     ID = ID + 1;
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
%     Nodes_TSP(ID).vertices = {xy}; 
%     Nodes_TSP(ID).theta = 3/2*pi;
%     Nodes_TSP(ID).TID = -1;
%     Nodes_TSP (ID).min_x = min(xy(:,1));
%     ID = ID + 1;
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
    
    
%     Nodes_TSP(ID).vertices = {xy}; 
%     Nodes_TSP(ID).theta = pi;
%     Nodes_TSP(ID).TID = -1;
%     Nodes_TSP (ID).min_x = min(xy(:,1));
%     ID = ID + 1;
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
%     Nodes_TSP(ID).vertices = {xy}; 
%     Nodes_TSP(ID).theta = pi/2;
%     Nodes_TSP(ID).TID = -1;
%     Nodes_TSP (ID).min_x = min(xy(:,1));
%     ID = ID + 1;
end



ID_05pi = ID - 1;


%% plot the smaller cells
ind_plot_tar = 1;

figure
axis equal

for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
    if any(xt(1)>w_xmax) || any(xt(2)>w_ymax)
        continue
    end
    
    plot(xt(1),xt(2),'r.','MarkerSize',30)  
%     text(xt(1),xt(2)-0.1,num2str(i))
    hold on
end


for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
    if any(x>w_xmax) || any(y>w_ymax)
        continue
    end
    
    k = convhull(x,y);
    fill(x(k),y(k),[0.5,0.5,0.5])
%     text(x,y-0.1,num2str(i))
    hold on
end


if ind_plot_tar == 1
    begin = ID_pi+1;
    finish = ID_05pi;
else
    if ind_plot_tar == 2
        begin = ID_15pi+1;
        finish = ID_pi;
    else
        if ind_plot_tar == 3
            begin = ID_2pi+1;
            finish = ID_15pi;
        else
            begin = 1;
            finish = ID_2pi;
        end
    end
end


% plot the rectangles
for i= begin : finish
    v = Nodes_TSP(i).vertices{1};
    v = v';
    plot(v(1,:),v(2,:),'k-')
    plot([v(1,1),v(1,4)],[v(2,1),v(2,4)],'k-')
    text(v(1,1)+0.1,v(2,1)+0.3,num2str(i));
    
end
for i=1:size(tar_rect)
    v = tar_rect(i, ind_plot_tar);
    v = v{1};
    if isempty(v)
        continue;
    end
    if any(v(1,:)>w_xmax) || any(v(2,:)>w_ymax)
        continue
    end
    
    fill(v(1,:),v(2,:),'b')
    
end



for i=1:size(walls,2)
    v = walls(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
%     if i==1
%         y(2:3) = 4.7;
%     end
%     if i==4
%         x(3:4) = 4.7;
%     end
    
    if any(x>w_xmax) || any(y>w_ymax)
        continue
    end
    
    k = convhull(x,y);
    fill(x(k),y(k),[0.5,0.5,0.5])
%     text(x,y-0.1,num2str(i))
    hold on
end
%  
% pv = [8,12,11];
% % pv = [21,36,31,26];
% for k=1 : length(pv)-1
%     i = pv(k);
%     v1 = Nodes_TSP(i).vertices{1};
%     v1 = v1';
%     x1 = mean(v1(1,:));
%     y1 = mean(v1(2,:));
%     j = pv(k+1);
%     v2 = Nodes_TSP(j).vertices{1};
%     v2 = v2';
%     x2 = mean(v2(1,:));
%     y2 = mean(v2(2,:));
%     plot([x1,x2],[y1,y2],'r-')
%     
%     text(v1(1,1)+0.1,v1(2,1)+0.1,num2str(i));
%     
% end
% text(v2(1,1)+0.1,v2(2,1)+0.1,num2str(j));


% title(['Result of Approximate Cell Decomposition with c\_max=',num2str(cell_max),', C-Targets at \theta=2\pi'])
set(gca,'XTick',[w_xmin : 0.5: w_xmax]);
set(gca,'YTick',[w_ymin : 0.5: w_ymax]);
axis equal
xlim([3.2,w_xmax])
ylim([3.4,5.8])


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
            for nCB = 1:length(obs_all)
                v = obs_all(nCB).vertices;
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

% 
% for idt=1:length(ind_t)
%     i = ind_t(idt);
%     TSP_nodes_Ti = [];
%      
%     for k = 1:4
%         t_theta = pi/2*k;
%         t = Rectangles_T(i, k);
%         tt = t{1};
%         if isempty(tt)
%             continue
%         end
%         
%         for j = 1:L0
%             is_conn = false;
%             v1 = Nodes_TSP(j).vertices;
%             R1 = v1(1);
%             R1 = R1{1}';
%             
%             if abs(Nodes_TSP(j).theta - t_theta) > pi/2
%                 continue
%             end
%             
%             
%             for m=1:size(tt, 2)
%                 ttt = tt(m);
%                 xy = ttt{1}';
%                 R2 = xy;
%                 
%                 
%                 % check if CT overlaps with CB
%                 is_CB = false;
%                 for nCB = 1:length(recFur)
%                     v = recFur(nCB).vertices;
%                     [out, area] =rect_relation(R1, R2);
%                     if out ~= 1
%                         is_CB = true;
%                     end
%                 end
%                 if is_CB
%                     continue
%                 end
%                 
%                 if rect_adj(R1, R2)
%                     if abs(t_theta - Nodes_TSP(j).theta) < 1e-6
%                         % Node i and Node j are on the same theta plane
%                         if abs(t_theta - pi/2) < 1e-6 || abs(t_theta - 3*pi/2) <1e-6
%                             % theta = pi/2 or 3*pi/2
%                             if min(R1(2,:)) >= max(R2(2,:)) || min(R2(2,:)) >= max(R1(2,:))
%                                 is_conn = true;
%                             end
%                         else
%                             % theta = pi or 2*pi
%                             if min(R1(1,:)) >= max(R2(1,:)) || min(R2(1,:)) >= max(R1(1,:))
%                                 is_conn = true;
%                             end
%                         end
%                     else
%                         is_conn = true;
%                     end 
%                 end
%                 if is_conn
%                     conn_pair = [conn_pair;Nodes_TSP(j).ID, ID];
%                 end
%             end
%             
%         end 
%         Nodes_TSP(ID).ID = ID;
%         t_xy = mean(ttt{1},1);
%         Nodes_TSP(ID).vertices = {xy'}; 
%         Nodes_TSP(ID).theta = t_theta;
%         Nodes_TSP(ID).TID = i;
%         Nodes_TSP (ID).min_x = t_xy(1);
%         TSP_nodes_Ti = [TSP_nodes_Ti, ID];
%         ID = ID + 1;
%     end
%     black_cells{idt} = TSP_nodes_Ti;
% end

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



% save conn_pair conn_pair
% save Nodes_TSP Nodes_TSP

%% Solve TSP
fprintf('TSP construction\n');

nStops = length(Nodes_TSP) + 1;
% black = [nStops-size(ind_t,2):nStops-1];
% flag = ismember([1:nStops],black);

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
        % modification: restrict directed arc at theta
%         ID_1 = Nodes_TSP(n1).ID;
%         ID_2 = Nodes_TSP(n2).ID;
%         if abs(Nodes_TSP(n1).theta - Nodes_TSP(n2).theta) > pi/2
%             continue
%         end
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
        
        
        
        % original code
%         distance_TSP(index) = conn(st);
%         idxs(index,:) = [n1, n2];
%         index = index + 1;
    else
        continue
%         distance(i) = 1e5;
%         not_conn = [not_conn,i];
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

high = [];
for i = 1:size(conn_pair,1)
    n1 = conn_pair(i, 1);
    n2 = conn_pair(i, 2);
    if n1>begin && n1<=final && n2>begin && n2<=final
        high = [high;n1,n2];
    end
    
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


% figure
% p = plot(G,'XData',x,'YData',y,'ZData',z);%,'EdgeLabel',G.Edges.Weight);
% p.EdgeColor = [0.9,0.9,0.9];
% highlight(p, [begin+1:final],'NodeColor','b','MarkerSize',6)
% highlight(p, [size(Nodes_TSP,2)-tot+1:size(Nodes_TSP,2)],'NodeColor','r','MarkerSize',6)
% highlight(p, high(:,1), high(:,2), 'EdgeColor','r','LineWidth',1.5)
% 
% 
% xlabel('X(m)')
% ylabel('Y(m)')
% zlabel('\theta(rad)')
% title('Connectivity Graph in Configuration Space')
% text(-4,-3,'Red: C-Targets, Green: Start, Magenta:dummy')







%%
fprintf('TSP constraints\n');

lendist = length(distance_TSP);
tsp = optimproblem;
trips = optimvar('trips',lendist,1,'Type','integer','LowerBound',0,'UpperBound',1);

tsp.Objective = distance_TSP'*trips;

% (1) in == out
constrips = optimconstr(nStops,1);
for stops = 1:nStops
    whichIdxs = (idxs == stops);
    constrips(stops) = sum(trips(whichIdxs(:,1))) == sum(trips(whichIdxs(:,2)));
end
tsp.Constraints.constrips = constrips;

% (2) visit dummy zero
constrtrips2 = optimconstr(1,1);
stops = nStops;
whichIdxs = (idxs == stops);
% constrtrips2(1) = sum(trips(whichIdxs(:,1))) == 3;
constrtrips2(1) = sum(trips(whichIdxs(:,1))) == 4;
tsp.Constraints.constrtrips2 = constrtrips2;

% (3) assign starting node
t_rm = [t2,t1,t3,t4];
% t_rm = [75, 65, 110];
dummy = nStops;
constrtrips_t2dummy = optimconstr(length(t_rm),1);
for i = 1:length(t_rm)
    start = t_rm(i);
    ind1 = (idxs==dummy);
    ind2 = (idxs==start);
    ind = ind1(:,1)&ind2(:,2);
    constrtrips_t2dummy(i) = sum(trips(ind)) == 1;
end
tsp.Constraints.constrtrips_t2dummy = constrtrips_t2dummy;

% (4) assign finish node
s_rm = [s2,s1,s4];
dummy = nStops;
constrtrips_dummy2s = optimconstr(length(s_rm),1);
for i = 1:length(s_rm)
    finish = s_rm(i);
    ind1 = (idxs==finish);
    ind2 = (idxs==dummy);
    ind = ind1(:,1)&ind2(:,2);
    constrtrips_dummy2s(i) = sum(trips(ind)) == 1;
end
tsp.Constraints.constrtrips_dummy2s = constrtrips_dummy2s;

% % (4) assign last node
% constrtrips4 = optimconstr(1,1);
% last = 64;
% %     last = 15; % Rm1
% %     last = 69; % Rm2
% %     last = 20; % Rm3
% dummy = nStops;
% ind1 = (idxs==last);
% ind2 = (idxs==dummy);
% ind = ind1(:,1)&ind2(:,2);
% constrtrips4(1) = sum(trips(ind)) == 1;
% tsp.Constraints.constrtrips4 = constrtrips4;

% ind_t_area3 = [27,20,8,10];
% ind_t_area2 = [25,6,29,14,28,4,1,26,24,13];
% ind_t_area1 = [12,17,11,30,16,2,28,15,5,3,19];
% ind_t_area = ind_t_area1;

% (5) visit black at least once
constr2trips = optimconstr(2*length(black_cells),1);
ind = 1;
for i = 1:length(black_cells)
    stops = black_cells{i};
    whichIdxs = (idxs == stops(1));
    for k = 2:length(stops)
        whichIdxs = whichIdxs | (idxs == stops(k));
    end
    
%     whichIdxs = any(whichIdxs,2); % start or end at stops
    constr2trips(i) = sum(trips(whichIdxs(:,1))) >= 1;
    constr2trips(length(black_cells)+i) = sum(trips(whichIdxs(:,2))) >= 1;
    ind = ind + 1;
end
tsp.Constraints.constr2trips = constr2trips;


% (6) t2, t1, t3
% t_rm = [97, 65, 110];
constrtrips_t_rm = optimconstr(2*length(t_rm),1);
for i = 1:length(t_rm)
    stops = t_rm(i);
    whichIdxs = (idxs == stops(1));
    constrtrips_t_rm(i) = sum(trips(whichIdxs(:,1))) == 1;
    constrtrips_t_rm(length(t_rm)+i) = sum(trips(whichIdxs(:,2))) == 1;
end
tsp.Constraints.constrtrips_t_rm = constrtrips_t_rm;

% (6) s4, s1, s2
% s_rm = [64,  99, 111];
constrtrips_s_rm = optimconstr(2*length(s_rm),1);
for i = 1:length(t_rm)
    stops = t_rm(i);
    whichIdxs = (idxs == stops(1));
    constrtrips_s_rm(i) = sum(trips(whichIdxs(:,1))) == 1;
    constrtrips_s_rm(length(t_rm)+i) = sum(trips(whichIdxs(:,2))) == 1;
end
tsp.Constraints.constrtrips_s_rm = constrtrips_s_rm;

%% plot the graph for updating subtours

s = zeros(1, size(conn_pair,1));
t = zeros(1, size(conn_pair,1));
weights = zeros(1, size(conn_pair,1));
x = zeros(1, size(Nodes_TSP,2));
y = zeros(1, size(Nodes_TSP,2));
z = zeros(1, size(Nodes_TSP,2));


for i = 1:size(conn_pair,1)
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
highlight(p, [ID_2pi+1:ID_15pi],'NodeColor','b','MarkerSize',6)
highlight(p, [size(Nodes_TSP,2)-tot+1:size(Nodes_TSP,2)],'NodeColor','r','MarkerSize',6)
% highlight(p,start, 'NodeColor','g','MarkerSize',6)
highlight(p, size(Nodes_TSP,2)+1, 'NodeColor','m','MarkerSize',6);

xlabel('X(m)')
ylabel('Y(m)')
zlabel('\theta(rad)')
title('Connectivity Graph in Configuration Space')
text(-4,-3,'Red: C-Targets, Green: Start, Magenta:dummy')






%% Subtours

fprintf('TSP solve begins \n');

opts = optimoptions('intlinprog','Display','off','Heuristics','advanced',...
    'LPPreprocess','basic');
% tspsol = solve(tsp,opts)
[tspsol,fval,exitflag,output] = solve(tsp,opts);

[tours,is_loop] = detectSubtours_3_middle(tspsol.trips,idxs);
numtours = num_subtours_new(tours); % number of subtours
flag_region = cons_region(tours, t1,s1,t2,s2,t4,s4);
% numtours = length(tours); % number of subtours
fprintf('# of subtours: %d\n',numtours);

% Index of added constraints for subtours
k = 1;
iter = 1;

while numtours ~= length(t_rm) || ~flag_region % repeat until there is just one subtour
    % Add the subtour constraints
    for ii = 1:numtours
        subTourIdx = tours{ii}; % Extract the current subtour
%         The next lines find all of the variables associated with the
%         particular subtour, then add an inequality constraint to prohibit
%         that subtour and all subtours that use those stops.
        variations = nchoosek(1:length(subTourIdx),2);
        a = false(length(idxs),1);
        for jj = 1:size(variations,1)
            % whichVar: the edge in the subtour, indexed by idxs
            whichVar = (sum(idxs==subTourIdx(variations(jj,1)),2)) & ...
                       (sum(idxs==subTourIdx(variations(jj,2)),2));
                   
            a = a | whichVar;
        end
        % a: all the edges in the subtour
        tsp.Constraints.(sprintf('subtourconstr%i',k)) = sum(trips(a)) <= length(subTourIdx)-1;
        k = k + 1;
    end
    % Try to optimize again
    [tspsol,fval,exitflag,output] = solve(tsp,opts);

    % Visualize result
    % [lh,lh_dummy] = updateSalesmanPlot_2(lh,lh_dummy,tspsol.trips,idxs,stopsLon,stopsLat);

    
%     save tsp_temp tsp
%     save trips_temp trips
%     save idxs_temp idxs
    
    % How many subtours this time?
    tours = detectSubtours_3_middle(tspsol.trips,idxs);
    numtours = num_subtours_new(tours); % number of subtours
    flag_region = cons_region(tours, t1,s1,t2,s2,t4,s4);
%     numtours = length(tours); % number of subtours
    fprintf('# of subtours: %d, ',numtours);
    fprintf('# of iterations: %d\n',iter);
    iter = iter + 1;
    
    % update plot
    p.EdgeColor = [0.9,0.9,0.9];
    trip_final = idxs(logical(tspsol.trips),:);
    highlight(p, trip_final(:,1),trip_final(:,2),'EdgeColor','r','LineWidth',1.5)
    pause(0.1);
end

%[lh,lh_dummy] = updateSalesmanPlot_2(lh,lh_dummy,tspsol.trips,idxs,stopsLon,stopsLat);

title('Solution with Subtours Eliminated');
hold off

disp(output.absolutegap)
fval
exitflag



%% plot the results
% clear all
% close all
% addpath('/Users/zhengmin321123/Desktop/cornellGD/pathPlanning(1)/classdef')
% 
% cirFur = cirObs([3,3],1,'furniture');
% plotObs(cirFur,'c'); %plotting obs
% lineX = [0  3]; lineY = [0  3]
% tf = ifLineCross(cirFur,lineX,lineY,'show') 
% 
% xlim = [0  9.2];
% ylim = [0  9.2];
% axis equal

s = zeros(1, size(conn_pair,1));
t = zeros(1, size(conn_pair,1));
weights = zeros(1, size(conn_pair,1));
x = zeros(1, size(Nodes_TSP,2));
y = zeros(1, size(Nodes_TSP,2));
z = zeros(1, size(Nodes_TSP,2));


for i = 1:size(conn_pair,1)
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
highlight(p, [1:size(Nodes_TSP,2)-tot],'NodeColor','b')%,'MarkerSize',6)
highlight(p, [size(Nodes_TSP,2)-tot+1:size(Nodes_TSP,2)],'NodeColor','r')%,'MarkerSize',6)
highlight(p,s_rm, 'NodeColor','b','MarkerSize',6)
highlight(p, size(Nodes_TSP,2)+1, 'NodeColor','m','MarkerSize',6);
trip_final = idxs(logical(tspsol.trips),:);
highlight(p, trip_final(:,1),trip_final(:,2),'EdgeColor','g','LineWidth',1.5)
highlight(p,t_rm, 'NodeColor','c','MarkerSize',6)
% if Room ~= 4
%     highlight(p,t_rm, 'NodeColor','c','MarkerSize',6)
% end

xlabel('X(m)')
ylabel('Y(m)')
zlabel('\theta(rad)')
title('Connectivity Graph in Configuration Space')
text(-4,-3,'Red: C-Targets, Green: Start, Magenta:dummy')


%%  plot the results

plot_grid = 0.3;

% before smoothing

figure
for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
    if any(xt > w_xmax)
        continue
    end
    
%     plot(xt(1),xt(2),'ko') 
    plot(xt(1),xt(2),'.r', 'MarkerSize',30)
    text(xt(1),xt(2)-0.1,num2str(i))
    hold on
end

% for i=1:size(walls,2)
%     v = walls(i).vertices;
%     x = round(v(1,:),1);
%     y = round(v(2,:),1);
%     if i==1
%         y(2:3) = 4.7;
%     end
%     if i==4
%         x(3:4) = 4.7;
%     end
%     
%     if any(x>4.9) || any(y>4.9)
%         continue
%     end
%     
%     k = convhull(x,y);
%     fill(x(k),y(k),[0.5,0.5,0.5])
% %     text(x,y-0.1,num2str(i))
%     hold on
% end

for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
    if any(x>w_xmax) || any(y>w_ymax)
        continue
    end
    
    k = convhull(x,y);
    fill(x(k),y(k),[0.5,0.5,0.5])
%     text(x,y-0.1,num2str(i))
    hold on
end



for i = 1:length(t_rm)
    it = t_rm(i);
    xs = mean(Nodes_TSP(it).vertices{1}(:,1));
    ys = mean(Nodes_TSP(it).vertices{1}(:,2));
    % text(xs,ys-0.1,'Start Position')
%     scatter(xs, ys, 'o', 'MarkerFaceColor', 'b')
%     plot(xs, ys,'bo')
    plot(xs,ys,'.b', 'MarkerSize',30)
    hold on
    plot_FOV(xs, ys, Nodes_TSP(it).theta, false);
    
end
for i = 1:length(s_rm)
    it = s_rm(i);
    xs = mean(Nodes_TSP(it).vertices{1}(:,1));
    ys = mean(Nodes_TSP(it).vertices{1}(:,2));
    % text(xs,ys-0.1,'Start Position')
%     scatter(xs, ys, 'o', 'MarkerFaceColor', 'b')
%     plot(xs, ys,'bo')
    plot(xs,ys,'.m', 'MarkerSize',30)
    hold on
    plot_FOV(xs, ys, Nodes_TSP(it).theta, false);
end

for i = 1:size(trip_final, 1)
    n1 = trip_final(i,1);
    n2 = trip_final(i,2);
    if n1 > length(Nodes_TSP) || n2 > length(Nodes_TSP)
        continue
    end
    contact = [];
    v1 = Nodes_TSP(n1).vertices{1};
    v2 = Nodes_TSP(n2).vertices{1};
    
    if n1<=ID_05pi && n2<=ID_05pi
        for j = 1:4
            [in,on] = inpolygon(v1(j,1),v1(j,2),v2(:,1),v2(:,2));
            if on
                contact = [contact;v1(j,1),v1(j,2)];
            end
        end
        for j = 1:4
            [in,on] = inpolygon(v2(j,1),v2(j,2),v1(:,1),v1(:,2));
            if on
                contact = [contact;v2(j,1),v2(j,2)];
            end
        end
        [C,ia,ic] = unique(contact,'rows');
        contact = C;
        if ~isempty(contact)
            x_med = mean(contact(:,1));
            y_med = mean(contact(:,2));
            x1 = mean(v1(:,1));
            y1 = mean(v1(:,2));
            x2 = mean(v2(:,1));
            y2 = mean(v2(:,2));
            plot([x1,x_med,x2],[y1,y_med,y2],'r-')
    %         text(x1,y1,num2str(n1))
    %         text(x2,y2,num2str(n2))
            hold on
            angle = atan2(y_med-y1, x_med-x1);
            % plot intermediate robots
            num_plot = floor(sqrt(sum(([x1,y1]-[x_med,y_med]).^2))/plot_grid);
            for m = 1:num_plot
                plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, false);
                hold on
            end
            angle = atan2(y2-y_med, x2-x_med);
            % plot intermediate robots
            num_plot = floor(sqrt(sum(([x_med,y_med]-[x2,y2]).^2))/plot_grid);
            for m = 1:num_plot
                plot_FOV(x_med+plot_grid*m*cos(angle), y_med+plot_grid*m*sin(angle), angle, false);
                hold on
            end
        else
            x1 = mean(v1(:,1));
            y1 = mean(v1(:,2));
            x2 = mean(v2(:,1));
            y2 = mean(v2(:,2));
            plot([x1,x2],[y1,y2],'r-')
            hold on
            x1 = mean(v1(:,1));
            y1 = mean(v1(:,2));
            x2 = mean(v2(:,1));
            y2 = mean(v2(:,2));
            plot([x1,x2],[y1,y2],'r-')
    %         text(x1,y1,num2str(n1))
    %         text(x2,y2,num2str(n2))
            hold on
            angle = atan2(y2-y1, x2-x1);
            % plot intermediate robots
            num_plot = floor(sqrt(sum(([x1,y1]-[x2,y2]).^2))/plot_grid);
            for m = 1:num_plot
                plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, false);
                hold on
            end
        end
        
    else
        x1 = mean(v1(:,1));
        y1 = mean(v1(:,2));
        x2 = mean(v2(:,1));
        y2 = mean(v2(:,2));
        plot([x1,x2],[y1,y2],'r-')
%         text(x1,y1,num2str(n1))
%         text(x2,y2,num2str(n2))
        hold on
        angle = atan2(y2-y1, x2-x1);
        % plot intermediate robots
        num_plot = floor(sqrt(sum(([x1,y1]-[x2,y2]).^2))/plot_grid);
        for m = 1:num_plot
            plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, false);
            hold on
        end
        
    end
    
    
end


% plot FOV
for  i = 1:size(trip_final, 1)
    n1 = trip_final(i,1);
    n2 = trip_final(i,2);
    if n1 > length(Nodes_TSP) || n2 > length(Nodes_TSP)
        continue
    end
    if n1 == 229
        temp  =1;
    end
    v1 = Nodes_TSP(n1).vertices{1};
    v2 = Nodes_TSP(n2).vertices{1};
    x1 = mean(v1(:,1));
    y1 = mean(v1(:,2));
    x2 = mean(v2(:,1));
    y2 = mean(v2(:,2));
    if n2>ID_05pi
        plot_FOV(x2, y2, Nodes_TSP(n2).theta, true);
        hold on
    else
        plot_FOV(x2, y2, Nodes_TSP(n2).theta, false);
    end
    
end
% title('Before Smoothing')
% title(['Optimal path with c\_max=',num2str(cell_max),', w\_t=',num2str(w_trans)])
axis equal

xlim([w_xmin,w_xmax])
ylim([w_ymin,w_ymax])
set(gca,'XTick',[0 : 1: x_max]);
set(gca,'YTick',[0 : 1: y_max]);


%% after smoothing
figure


for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
    if any(xt > w_xmax)
        continue
    end
    
%     plot(xt(1),xt(2),'ko') 
    plot(xt(1),xt(2),'.r', 'MarkerSize',30)
    text(xt(1),xt(2)-0.1,num2str(ind_t(i)))
    hold on
end

for i=1:size(walls,2)
    v = walls(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
%     if i==1
%         y(2:3) = 4.7;
%     end
%     if i==4
%         x(3:4) = 4.7;
%     end
    
    if any(x>4.7) || any(y>4.7)
        continue
    end
    
    k = convhull(x,y);
    fill(x(k),y(k),[0.5,0.5,0.5])
%     text(x,y-0.1,num2str(i))
    hold on
end

for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
    if any(x>w_xmax) || any(y>w_ymax)
        continue
    end
    
    k = convhull(x,y);
    fill(x(k),y(k),[0.5,0.5,0.5])
%     text(x,y-0.1,num2str(i))
    hold on
end


fprintf('Smooth begins \n')

for m = 1:length(t_rm)
    
trip_temp = trip_final;
trip_raw = [];
curr = length(Nodes_TSP) + 1;
row_all = find(trip_temp(:,1)==curr);
row = row_all(m);
curr = trip_temp(row,2);
trip_temp(row,:) = [];

ind = 1;
while curr ~= length(Nodes_TSP) + 1
    trip_raw(ind) = curr;
    ind = ind + 1;
    row = find(trip_temp(:,1)==curr);
    row = row(1);
    curr = trip_temp(row,2);
    trip_temp(row,:) = [];
end

theta_start = Nodes_TSP(start).theta;
cost = cost_eval(w_trans,trip_raw,Nodes_TSP,theta_start);
fprintf(['Cost before smoothing: ',num2str(cost),'\n'])


% smooth the path


obs_ind = [1:length(obs_all)];


% values = ones(length(ind_t),1);
% T_index = containers.Map(ind_t, values);
Ts = [];
for i = 1:length(trip_raw)
    n = trip_raw(i);
    if n > ID_05pi
        Ts = [Ts,i];
    end
end

if isempty(Ts) || Ts(1) ~= 1
    Ts = [1,Ts];
end

if Ts(length(Ts)) ~= length(trip_raw)
    Ts = [Ts, length(trip_raw)];
end

trip_index = [1:length(trip_raw)];

for i = 1:length(Ts)-1
    s = Ts(i);
    e = Ts(i+1);
    if e-s < 2
        continue
    end
    for j = s:e-2
        for k = s+2:e
            if trip_index(j) == -1 || trip_index(k) == -1
                continue
            end
            n1 = trip_raw(j);
            n2 = trip_raw(k);
            v1 = Nodes_TSP(n1).vertices{1};
            v2 = Nodes_TSP(n2).vertices{1};
            x1 = mean(v1(:,1));
            y1 = mean(v1(:,2));
            x2 = mean(v2(:,1));
            y2 = mean(v2(:,2));
            xy1 = [x1,y1];
            xy2 = [x2,y2];

            
            if is_collisionfree([1:length(recFur)],recFur, xy1,xy2) && is_collisionfree([1:length(walls)],walls, xy1,xy2)
                trip_index(j+1:k-1) = -1;
            end
        
        
        end
    end
end

% trip_index = trip_index(trip_index ~= -1);

trip_raw = trip_raw( trip_index(trip_index ~= -1));




% % values = ones(length(ind_t),1);
% % T_index = containers.Map(ind_t, values);
% 
% while true
%     flag = false;
%     i = 1;
%     while i<length(trip_raw)-2
%         if trip_raw(i+1)>ID_05pi
%             % cannot bypass the target node
%             i = i+1;
%             continue
%         end
%         n1 = trip_raw(i);
%         n2 = trip_raw(i+2);
%         v1 = Nodes_TSP(n1).vertices{1};
%         v2 = Nodes_TSP(n2).vertices{1};
%         x1 = mean(v1(:,1));
%         y1 = mean(v1(:,2));
%         x2 = mean(v2(:,1));
%         y2 = mean(v2(:,2));
%         xy1 = [x1,y1];
%         xy2 = [x2,y2];
%         if is_collisionfree(obs_ind,obs_all, xy1,xy2)
%             trip_raw(i+1) = [];
%             flag = true;
%         end
%         
%         i = i+1;
%     end
%     if flag == false
%         break;
%     end
%     
%     
% end






% plot
for i = 1:length(trip_raw)-1
    n1 = trip_raw(i);
    n2 = trip_raw(i+1);
    if n1 > length(Nodes_TSP) || n2 > length(Nodes_TSP)
        continue
    end
    contact = [];
    v1 = Nodes_TSP(n1).vertices{1};
    v2 = Nodes_TSP(n2).vertices{1};
    
    if n1<=ID_05pi && n2<=ID_05pi
        for j = 1:4
            [in,on] = inpolygon(v1(j,1),v1(j,2),v2(:,1),v2(:,2));
            if on
                contact = [contact;v1(j,1),v1(j,2)];
            end
        end
        for j = 1:4
            [in,on] = inpolygon(v2(j,1),v2(j,2),v1(:,1),v1(:,2));
            if on
                contact = [contact;v2(j,1),v2(j,2)];
            end
        end
        if ~isempty(contact)
            [C,ia,ic] = unique(contact,'rows');
            contact = C;
            x_med = mean(contact(:,1));
            y_med = mean(contact(:,2));
            x1 = mean(v1(:,1));
            y1 = mean(v1(:,2));
            x2 = mean(v2(:,1));
            y2 = mean(v2(:,2));
            plot([x1,x_med,x2],[y1,y_med,y2],'r-')
            hold on
            angle = atan2(y_med-y1, x_med-x1);
            % plot intermediate robots
            num_plot = floor(sqrt(sum(([x1,y1]-[x_med,y_med]).^2))/plot_grid);
            for m = 1:num_plot
                plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, false);
                hold on
            end
            angle = atan2(y2-y_med, x2-x_med);
            % plot intermediate robots
            num_plot = floor(sqrt(sum(([x_med,y_med]-[x2,y2]).^2))/plot_grid);
            for m = 1:num_plot
                plot_FOV(x_med+plot_grid*m*cos(angle), y_med+plot_grid*m*sin(angle), angle, false);
                hold on
            end
        else
            x1 = mean(v1(:,1));
            y1 = mean(v1(:,2));
            x2 = mean(v2(:,1));
            y2 = mean(v2(:,2));
            plot([x1,x2],[y1,y2],'r-')
            hold on
            x1 = mean(v1(:,1));
            y1 = mean(v1(:,2));
            x2 = mean(v2(:,1));
            y2 = mean(v2(:,2));
            plot([x1,x2],[y1,y2],'r-')
    %         text(x1,y1,num2str(n1))
    %         text(x2,y2,num2str(n2))
            hold on
            angle = atan2(y2-y1, x2-x1);
            % plot intermediate robots
            num_plot = floor(sqrt(sum(([x1,y1]-[x2,y2]).^2))/plot_grid);
            for m = 1:num_plot
                plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, false);
                hold on
            end
        end
%         text(x1,y1,num2str(n1))
%         text(x2,y2,num2str(n2))
        hold on
    else
        x1 = mean(v1(:,1));
        y1 = mean(v1(:,2));
        x2 = mean(v2(:,1));
        y2 = mean(v2(:,2));
        plot([x1,x2],[y1,y2],'r-')
%         text(x1,y1,num2str(n1))
%         text(x2,y2,num2str(n2))
        hold on
        angle = atan2(y2-y1, x2-x1);
        % plot intermediate robots
        num_plot = floor(sqrt(sum(([x1,y1]-[x2,y2]).^2))/plot_grid);
        for m = 1:num_plot
            plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, false);
            hold on
        end
    end
    
    
end

v1 = Nodes_TSP(start).vertices{1};
x1 = mean(v1(:,1));
y1 = mean(v1(:,2));
plot_FOV(x1, y1, Nodes_TSP(start).theta, false);
% plot FOV
for  i = 1:length(trip_raw)-1
    n1 = trip_raw(i);
    n2 = trip_raw(i+1);
    if n1 > length(Nodes_TSP) || n2 > length(Nodes_TSP)
        continue
    end
    v1 = Nodes_TSP(n1).vertices{1};
    v2 = Nodes_TSP(n2).vertices{1};
    x1 = mean(v1(:,1));
    y1 = mean(v1(:,2));
    x2 = mean(v2(:,1));
    y2 = mean(v2(:,2));
    
    if n2>ID_05pi
        plot_FOV(x2, y2, Nodes_TSP(n2).theta, true);
        hold on
%     else
%         plot_FOV(x2, y2, Nodes_TSP(n2).theta, false);
    end
    
end

end



% title('After Smoothing')
axis equal
xlim([w_xmin,w_xmax])
ylim([w_ymin,w_ymax])
set(gca,'XTick',[0 : 0.5: x_max]);
set(gca,'YTick',[0 : 0.5: y_max]);

cost = cost_eval(w_trans,trip_raw,Nodes_TSP,theta_start);

fprintf(['Cost after smoothing: ',num2str(cost),'\n'])



for i = 1:length(t_rm)
    it = t_rm(i);
    xs = mean(Nodes_TSP(it).vertices{1}(:,1));
    ys = mean(Nodes_TSP(it).vertices{1}(:,2));
    % text(xs,ys-0.1,'Start Position')
%     scatter(xs, ys, 'o', 'MarkerFaceColor', 'b')
%     plot(xs, ys,'bo')
    plot(xs,ys,'.c', 'MarkerSize',30)
    hold on
    plot_FOV(xs, ys, Nodes_TSP(it).theta, false);
end
for i = 1:length(s_rm)
    it = s_rm(i);
    xs = mean(Nodes_TSP(it).vertices{1}(:,1));
    ys = mean(Nodes_TSP(it).vertices{1}(:,2));
    % text(xs,ys-0.1,'Start Position')
%     scatter(xs, ys, 'o', 'MarkerFaceColor', 'b')
%     plot(xs, ys,'bo')
    plot(xs,ys,'.m', 'MarkerSize',30)
    hold on
    plot_FOV(xs, ys, Nodes_TSP(it).theta, false);
end


xlim([3.2,6]); ylim([3.2,6]);
%% save the path segments
Nodes_path_middle = {};
ind = 1;

for i = 1:length(trip_raw)
    n1 = trip_raw(i);
    if n1 > length(Nodes_TSP)
        continue
    end
    v1 = Nodes_TSP(n1).vertices{1};
    x1 = mean(v1(:,1));
    y1 = mean(v1(:,2));
    node.vertices = {[x1,y1]};
    node.theta = Nodes_TSP(n1).theta;
    if n1>ID_05pi
        node.isCT = true;
    else
        node.isCT = false;
    end
    Nodes_path_middle(ind) = {node};
    ind = ind+1;
end

% save Nodes_path_Rm1 Nodes_path_Rm1

figure
for  i = 1:length(Nodes_path_middle)-1
    n1 = Nodes_path_middle{i};
    n2 = Nodes_path_middle{i+1};
    v1 = n1.vertices{1};
    v2 = n2.vertices{1};
    x1 = mean(v1(:,1));
    y1 = mean(v1(:,2));
    x2 = mean(v2(:,1));
    y2 = mean(v2(:,2));
    plot([x1,x2], [y1,y2], 'b-')
    hold on
    if n2.isCT
        plot_FOV(x2, y2, n2.theta, true);
        hold on
%     else
%         plot_FOV(x2, y2, Nodes_TSP(n2).theta, false);
    end
end


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

