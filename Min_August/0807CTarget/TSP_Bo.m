% clear
close all
clc

addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Min_August\0807CTarget\infoData')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Min_August\0807CTarget\classdef')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\helperFuns')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Rectangle_Decomposition')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\Sort_struct')
addpath('C:\Users\LISC\Documents\Zeyu Liu research documents\Workspace_Min\Mac\Min_Workspace\pathPlanning - Copy\TSP_Matlab')


%%
w_trans = 0.3;
w_rotat = 1 - w_trans;

cell_max = 9;

warning off

Room = 4;
w_xmin = 0;
w_xmax = 9.2;
w_ymin = 0;
w_ymax = 9.2;


x_max = w_xmax;
y_max = w_ymax;

resolution_x = 0.2;
resolution_theta = pi/2;

%%%%%%%% MAIN
global UGV
[targets,walls,recFur,cirFur,UGV] = buildAll();
obs_all = [recFur,walls];

ind_t = [1:30];

load('tour_Heu.mat')


start = 31;

figure

for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
%     if any(xt > w_xmax)
%         continue
%     end
    
%     plot(xt(1),xt(2),'ko') 
    plot(xt(1),xt(2),'.r', 'MarkerSize',20)
    text(xt(1),xt(2)-0.1,num2str(i))
    hold on
end

for i=1:size(walls,2)
    v = walls(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
    k = convhull(x,y);
    fill(x(k),y(k),'k')
%     text(x,y-0.1,num2str(i))
    hold on
end

for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);  
    k = convhull(x,y);
    fill(x(k),y(k),'k')
%     text(x,y-0.1,num2str(i))
    hold on
end

xs = 6.5;
ys = 2;
% text(xs,ys-0.1,'Start Position')
scatter(xs, ys, 'o', 'MarkerFaceColor', 'b')
plot(xs, ys,'bo')
plot(xs,ys,'.b', 'MarkerSize',15)
hold on
plot_FOV(xs, ys, pi, false);

plot_grid = 0.3;

tour_Heu = tour_Heu(tour_Heu<=30);
tour_Heu = fliplr(tour_Heu);
for i = 1:length(tour_Heu)-1
    n1 = tour_Heu(i);
    n2 = tour_Heu(i+1);
    vt1 = targets(n1).loc;
    vt2 = targets(n2).loc;
    plot([vt1(1),vt2(1)],[vt1(2),vt2(2)],'r-')
    hold on
%     plot_FOV(vt2(1), vt292), Nodes_TSP(n2).theta, true);
    x1 = vt1(1);
    y1 = vt1(2);
    x2 = vt2(1);
    y2 = vt2(2);
    angle = atan2(y2-y1, x2-x1);
    % plot intermediate robots
    num_plot = floor(sqrt(sum(([x1,y1]-[x2,y2]).^2))/plot_grid);
    for m = 1:num_plot
        if num_plot > 2
            if m == num_plot-1
                flag_FOV = true;
            else
                flag_FOV = false;
            end
        else
            if m == num_plot-1
                flag_FOV = true;
            else
                flag_FOV = false;
            end
        end
        plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, flag_FOV);
        hold on
    end
end

vt1 = [6.5,2];
vt2 = targets(tour_Heu(1)).loc;
plot([vt1(1),vt2(1)],[vt1(2),vt2(2)],'r-')
hold on
%     plot_FOV(vt2(1), vt292), Nodes_TSP(n2).theta, true);
x1 = vt1(1);
y1 = vt1(2);
x2 = vt2(1);
y2 = vt2(2);
angle = atan2(y2-y1, x2-x1);
% plot intermediate robots
num_plot = floor(sqrt(sum(([x1,y1]-[x2,y2]).^2))/plot_grid);
for m = 1:num_plot
    if num_plot > 2
        if m == num_plot-1
            flag_FOV = true;
        else
            flag_FOV = false;
        end
    else
        if m == num_plot -1
            flag_FOV = true;
        else
            flag_FOV = false;
        end
    end
    plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, flag_FOV);
    hold on
end



axis equal
xlim([w_xmin,w_xmax])
ylim([w_ymin,w_ymax])
set(gca,'XTick',[0 : 1: x_max]);
set(gca,'YTick',[0 : 1: y_max]);

%%

w_trans = 0.3;
Nodes_path = {};
ind = 1;
v = [6.5,2];
node.vertices = {mean(v,1)};
node.ID = -1;
Nodes_path(ind) = {node};
ind = ind+1;

tour_Heu = [10,27,20,8,7,19,25,13,6,26,4,29,14,28,1,24,22,18,21,9,2,11,30,16,17,12,15,5,3,23];
tour_Heu = [10,27,13,25,10,7,6,26,4,29,14,28,1,24,13,27,20,8,7,22,18,21,9,2,11,30,16,17,12,15,5,3,22];;
for i = 1:length(tour_Heu)
    n1 = tour_Heu(i);
    v = targets(n1).loc;
    node.vertices = {mean(v,1)};
    node.ID = n1;
    Nodes_path(ind) = {node};
    ind = ind+1;
    
end





[cost,trans_tot,r_tot] = cost_eval_3(w_trans,Nodes_path,pi)
%% 
% small map 3

w_trans = 0.3;
w_rotat = 1 - w_trans;

cell_max = 9;

warning off

w_xmin = 0;
w_xmax = 5;
w_ymin = 0;
w_ymax = 5;


x_max = w_xmax;
y_max = w_ymax;

resolution_x = 0.2;
resolution_theta = pi/2;

%%%%%%%% MAIN
global UGV
[targets,walls,recFur,cirFur,UGV] = buildAll();
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

% workspace 2
targets(3).loc = [2.8, 3];
targets(5).loc(2) = 2.5;
targets(9).loc(1) = 4;
targets(12).loc = [0.5,4];
targets(15).loc(2) = 1.5;
targets(21).loc(2) = 0.8;

recFur(3).vertices = [recFur(3).vertices(1,:)-1;recFur(3).vertices(2,:)-1.5];
recFur(4).vertices = [recFur(4).vertices(1,:)-5;recFur(4).vertices(2,:)-2];
recFur(1).vertices = [recFur(1).vertices(1,:)+1;recFur(1).vertices(2,:)+1];

walls(6).vertices(1,3:4) = [4.7,4.7];
walls(5).vertices(2,2:3) = [4.7,4.7];

obs_all = [recFur,walls];

ind_t = [12,18,22,21,9,7,3,5,15];
% load('tour_Heu_map3.mat')
load('tour_Heu_map1')


tour_Heu = tour_Heu(tour_Heu<=9);
w_trans = 0.3;
Nodes_path = {};
ind = 1;
v = [4.3,1.3];
node.vertices = {mean(v,1)};
node.ID = -1;
Nodes_path(ind) = {node};
ind = ind+1;

for i = 1:length(tour_Heu)
    n1 = tour_Heu(i);
    v = targets(ind_t(n1)).loc;
    node.vertices = {mean(v,1)};
    node.ID = n1;
    node.FOV = true;
    Nodes_path(ind) = {node};
    ind = ind+1;
     
%     if n1==6
%         v = [3.7,4.3];
%         node.vertices = {mean(v,1)};
%         node.ID = n1;
%         node.FOV = false;
%         Nodes_path(ind) = {node};
%         ind = ind+1;
%         
%     end
%     if n1==9
%         v = [0.5,0.7];
%         node.vertices = {mean(v,1)};
%         node.ID = n1;
%         node.FOV = false;
%         Nodes_path(ind) = {node};
%         ind = ind+1;
%         
%         v = [1.1,0.7];
%         node.vertices = {mean(v,1)};
%         node.ID = n1;
%         node.FOV = false;
%         Nodes_path(ind) = {node};
%         ind = ind+1;
%     end
%     if n1==5
%         v = [3.6,2.8];
%         node.vertices = {mean(v,1)};
%         node.ID = n1;
%         node.FOV = false;
%         Nodes_path(ind) = {node};
%         ind = ind+1;
%         
%     end
end





[cost,trans_tot,r_tot] = cost_eval_3(w_trans,Nodes_path,pi/2)

figure

for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
    plot(xt(1),xt(2),'.r', 'MarkerSize',20)
    text(xt(1),xt(2)-0.1,num2str(i))
    hold on
end

% for i=1:size(walls,2)
%     v = walls(i).vertices;
%     x = round(v(1,:),1);
%     y = round(v(2,:),1);
%     k = convhull(x,y);
%     fill(x(k),y(k),'k')
% %     text(x,y-0.1,num2str(i))
%     hold on
% end

for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);  
    k = convhull(x,y);
    fill(x(k),y(k),'k')
%     text(x,y-0.1,num2str(i))
    hold on
end

xs = 4.3;
ys = 1.3;
% text(xs,ys-0.1,'Start Position')
scatter(xs, ys, 'o', 'MarkerFaceColor', 'b')
plot(xs, ys,'bo')
plot(xs,ys,'.b', 'MarkerSize',15)
hold on
plot_FOV(xs, ys, pi/2, false);

plot_grid = 0.3;


% tour_Heu = fliplr(tour_Heu);
for i = 1:length(Nodes_path)-1
    vt1 = Nodes_path{i}.vertices{1};
    vt2 = Nodes_path{i+1}.vertices{1};
    plot([vt1(1),vt2(1)],[vt1(2),vt2(2)],'r-')
    hold on
%     plot_FOV(vt2(1), vt292), Nodes_TSP(n2).theta, true);
    x1 = vt1(1);
    y1 = vt1(2);
    x2 = vt2(1);
    y2 = vt2(2);
    angle = atan2(y2-y1, x2-x1);
    % plot intermediate robots
    num_plot = floor(sqrt(sum(([x1,y1]-[x2,y2]).^2))/plot_grid);
    for m = 1:num_plot
        if num_plot > 2
            if m == num_plot-1
                flag_FOV = true;
            else
                flag_FOV = false;
            end
        else
            if m == num_plot-1
                flag_FOV = true;
            else
                flag_FOV = false;
            end
        end
        flag_FOV = flag_FOV & Nodes_path{i+1}.FOV;
        plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, flag_FOV);
        hold on
    end
end
plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, true);

vt1 = [4.3,1.3];
vt2 = targets(ind_t(tour_Heu(1))).loc;
plot([vt1(1),vt2(1)],[vt1(2),vt2(2)],'r-')
hold on

plot_FOV(x1, y1, angle, flag_FOV);

%     plot_FOV(vt2(1), vt292), Nodes_TSP(n2).theta, true);
x1 = vt1(1);
y1 = vt1(2);
x2 = vt2(1);
y2 = vt2(2);
angle = atan2(y2-y1, x2-x1);
% plot intermediate robots
num_plot = floor(sqrt(sum(([x1,y1]-[x2,y2]).^2))/plot_grid);
% for m = 1:num_plot
%     if num_plot > 2
%         if m == num_plot-1
%             flag_FOV = true;
%         else
%             flag_FOV = false;
%         end
%     else
%         if m == num_plot -1
%             flag_FOV = true;
%         else
%             flag_FOV = false;
%         end
%     end
%     flag_FOV = flag_FOV & Nodes_path{ind_t(tour_Heu(1))}.FOV;
%     plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, flag_FOV);
%     hold on
% end



axis equal
xlim([0,5])
ylim([0,5])
set(gca,'XTick',[0 : 1: x_max]);
set(gca,'YTick',[0 : 1: y_max]);


%%
figure

for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
    plot(xt(1),xt(2),'.r', 'MarkerSize',20)
    text(xt(1),xt(2)-0.1,num2str(i))
    hold on
end

% for i=1:size(walls,2)
%     v = walls(i).vertices;
%     x = round(v(1,:),1);
%     y = round(v(2,:),1);
%     k = convhull(x,y);
%     fill(x(k),y(k),'k')
% %     text(x,y-0.1,num2str(i))
%     hold on
% end

for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);  
    k = convhull(x,y);
    fill(x(k),y(k),'k')
%     text(x,y-0.1,num2str(i))
    hold on
end

xs = 4.3;
ys = 1.3;
% text(xs,ys-0.1,'Start Position')
scatter(xs, ys, 'o', 'MarkerFaceColor', 'b')
plot(xs, ys,'bo')
plot(xs,ys,'.b', 'MarkerSize',15)
hold on
plot_FOV(xs, ys, pi, false);

plot_grid = 0.3;

t_pairs = [3,5;6,8;11,14];
t_pairs = [6,8;6,9;6,10;2,6;5,6;3,6];
% tour_Heu = fliplr(tour_Heu);
for i = 1:size(t_pairs,1)
    n1 = t_pairs(i,1);
    n2 = t_pairs(i,2);
    vt1 = Nodes_path{n1}.vertices{1};
    vt2 = Nodes_path{n2}.vertices{1};
    plot([vt1(1),vt2(1)],[vt1(2),vt2(2)],'r-')
    hold on
%     plot_FOV(vt2(1), vt292), Nodes_TSP(n2).theta, true);
    x1 = vt1(1);
    y1 = vt1(2);
    x2 = vt2(1);
    y2 = vt2(2);
    angle = atan2(y2-y1, x2-x1);
    % plot intermediate robots
    num_plot = floor(sqrt(sum(([x1,y1]-[x2,y2]).^2))/plot_grid);
    for m = 1:num_plot
        plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, false);
        hold on
    end
end

axis equal
xlim([0,5])
ylim([0,5])
set(gca,'XTick',[0 : 1: x_max]);
set(gca,'YTick',[0 : 1: y_max]);

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
