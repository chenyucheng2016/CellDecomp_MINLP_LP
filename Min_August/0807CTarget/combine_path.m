clear 
close all
clc

% addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy')
% addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Min_August\0807CTarget\infoData')
% addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Min_August\0807CTarget\classdef')
% addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\helperFuns')
% addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Rectangle_Decomposition')
% addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Sort_struct')
% addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\TSP_Matlab')

addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy')
addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/Min_August/0807CTarget/infoData')
addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/Min_August/0807CTarget/classdef')
addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/helperFuns')
addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/Rectangle_Decomposition')
addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/Sort_struct')
addpath('/Users/liuzeyu/Documents/Research/Min_Workspace/pathPlanning - Copy/TSP_Matlab')


w_trans = 0.3;
w_rotat = 1 - w_trans;

cell_max = 3;

warning off

Room = 3;

w_xmin = 0;
w_xmax = 9;
w_ymin = 0;
w_ymax = 9;


x_max = w_xmax;
y_max = w_ymax;

resolution_x = 0.2;
resolution_theta = pi/2;

%%%%%%%% MAIN
global UGV
[targets,walls,recFur,cirFur,UGV] = buildAll();
obs_all = [recFur,walls];



ind_t = [1:length(targets)];


load('Nodes_path_Rm1.mat')
load('Nodes_path_Rm2.mat')
load('Nodes_path_Rm3.mat')
load('Nodes_path_Rm4.mat')


plot_grid = 0.3;


Nodes_path_all = {};
ind = 1;




for i = 1:length(Nodes_path_Rm3)
%     if i==2 || i==5
%         continue
%     end
    node = Nodes_path_Rm3{i};
    Nodes_path_all(ind) = {node};
    ind = ind+1;
end

for i = 1:length(Nodes_path_Rm2)
%     if i > 12
%         continue
%     end
    node = Nodes_path_Rm2{i};
    Nodes_path_all(ind) = {node};
    ind = ind+1;
end

for i = 1:length(Nodes_path_Rm1)
%     if i ==1 || i > 13
%         continue
%     end
    node = Nodes_path_Rm1{i};
    Nodes_path_all(ind) = {node};
    ind = ind+1;
end

for i = 1:length(Nodes_path_Rm4)
%     if i<2
%         continue
%     end
    node = Nodes_path_Rm4{i};
    Nodes_path_all(ind) = {node};
    ind = ind+1;
end


cost = cost_eval_2(w_trans,Nodes_path_all,pi);
fprintf(['Cost before smoothing: ',num2str(cost),'\n'])

%%
figure

for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
    if any(xt > w_xmax)
        continue
    end
    
%     plot(xt(1),xt(2),'ko') 
    plot(xt(1),xt(2),'.r', 'MarkerSize',20)
    text(xt(1),xt(2)-0.1,num2str(i))
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
    
%     if any(x>4.7) || any(y>4.7)
%         continue
%     end
    
    k = convhull(x,y);
    f = fill(x(k),y(k),'k');
    alpha(f,1);
%     text(x,y-0.1,num2str(i))
    hold on
end

for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
%     if any(x>w_xmax) || any(y>w_ymax)
%         continue
%     end
    
    k = convhull(x,y);
    f = fill(x(k),y(k),[0,0,0]);
    alpha(f,1)
%     text(x,y-0.1,num2str(i))
    hold on
end




n1 = Nodes_path_Rm3{1};
v1 = n1.vertices{1};
x1 = mean(v1(:,1));
y1 = mean(v1(:,2));
plot_FOV(x1, y1, pi/2, false);
plot(x1, y1, 'b.','MarkerSize',15);
hold on

for  i = 1:length(Nodes_path_all)-1
    n1 = Nodes_path_all{i};
    n2 = Nodes_path_all{i+1};
    v1 = n1.vertices{1};
    v2 = n2.vertices{1};
    x1 = mean(v1(:,1));
    y1 = mean(v1(:,2));
    x2 = mean(v2(:,1));
    y2 = mean(v2(:,2));
    
    if n2.isCT
        plot_FOV(x2, y2, n2.theta, true);
        hold on
    end

    plot([x1,x2], [y1,y2], 'r-')
    hold on
%     text(x1,x2,num2str(i))
    angle = atan2(y2-y1, x2-x1);
    % plot intermediate robots
    num_plot = floor(sqrt(sum(([x1,y1]-[x2,y2]).^2))/plot_grid);
    for m = 1:num_plot
        plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, false);
        hold on
    end
end

axis equal
xlim([0, 9.2])
ylim([0, 9.2])
%% smoothing

% smooth the path
obs_ind = [1:25];

% values = ones(length(ind_t),1);
% T_index = containers.Map(ind_t, values);
Ts = [];
for i = 1:length(Nodes_path_all)
    if Nodes_path_all{i}.isCT
        Ts = [Ts,i];
    end
end

if Ts(1) ~= 1
    Ts = [1,Ts];
end

if Ts(length(Ts)) ~= length(Nodes_path_all)
    Ts = [Ts, length(Nodes_path_all)];
end

trip_index = [1:length(Nodes_path_all)];

for i = 1:length(Ts)-1
    s = Ts(i);
    e = Ts(i+1);
    if e-s < 2
        continue
    end
    for j = s:e-2
        for k = s+2:e
            if j==14 && k==16
                tem = 1;
            end
            
            n1 = Nodes_path_all{j};
            n2 = Nodes_path_all{k};
            if trip_index(j) == -1 || trip_index(k) == -1
                continue
            end
            v1 = n1.vertices{1};
            v2 = n2.vertices{1};
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

Nodes_path_all_smooth = Nodes_path_all( trip_index(trip_index ~= -1));

[cost,trans_tot,r_tot] = cost_eval_3(w_trans,Nodes_path_all_smooth,pi)
fprintf(['Cost after smoothing: ',num2str(cost),'\n'])


%%
figure

for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
    if any(xt > w_xmax)
        continue
    end
    
%     plot(xt(1),xt(2),'ko') 
    plot(xt(1),xt(2),'.r', 'MarkerSize',20)
    text(xt(1),xt(2)-0.1,num2str(i))
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
    
%     if any(x>4.7) || any(y>4.7)
%         continue
%     end
    
    k = convhull(x,y);
    f = fill(x(k),y(k),'k');
    alpha(f,1);
%     text(x,y-0.1,num2str(i))
    hold on
end

for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
%     if any(x>w_xmax) || any(y>w_ymax)
%         continue
%     end
    
    k = convhull(x,y);
    f = fill(x(k),y(k),[0,0,0]);
    alpha(f,1)
%     text(x,y-0.1,num2str(i))
    hold on
end




n1 = Nodes_path_Rm3{1};
v1 = n1.vertices{1};
x1 = mean(v1(:,1));
y1 = mean(v1(:,2));
plot_FOV(x1, y1, pi/2, false);
plot(x1, y1, 'b.','MarkerSize',15);
hold on

for  i = 1:length(Nodes_path_all_smooth)-1
    n1 = Nodes_path_all_smooth{i};
    n2 = Nodes_path_all_smooth{i+1};
    v1 = n1.vertices{1};
    v2 = n2.vertices{1};
    x1 = mean(v1(:,1));
    y1 = mean(v1(:,2));
    x2 = mean(v2(:,1));
    y2 = mean(v2(:,2));
    
    if n2.isCT
        plot_FOV(x2, y2, n2.theta, true);
        hold on
    end

    plot([x1,x2], [y1,y2], 'r-')
    hold on
%     text(x1,x2,num2str(i))
    angle = atan2(y2-y1, x2-x1);
    % plot intermediate robots
    num_plot = floor(sqrt(sum(([x1,y1]-[x2,y2]).^2))/plot_grid);
    for m = 1:num_plot
        plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, false);
        hold on
    end
end

axis equal
xlim([0, 9.2])
ylim([0, 9.2])


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

