%add camera FOV manually
close all
addpath(genpath('Targets/'));
addpath('Min_August\0807CTarget\infoData')
addpath('Min_August\0807CTarget\classdef')
addpath('helperFuns')
addpath('Rectangle_Decomposition')
addpath('Sort_struct')
addpath('TSP_Matlab')
addpath('Min_August\0807CTarget')
load('workspace.mat')
global ugv_h  ugv_w
ugv_h = 0.08; ugv_w = 0.18;
figure
for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
    plot(xt(1),xt(2),'.r', 'MarkerSize',20)
    text(xt(1),xt(2)-0.1,num2str(i))
    hold on
end
for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
    
    k = convhull(x,y);
    fill(x(k),y(k),'k')
    hold on
end
for i=1:length(walls)
    v = walls(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
    
    k = convhull(x,y);
    fill(x(k),y(k),'k')
    hold on
end
xlim([0,40])
ylim([0,40])
optimal_xy_unique = [];

for i = 1:size(optPath_xy_smooth,1)
    cur_line = optPath_xy_smooth(i,:);
    next_line = optPath_xy_smooth(min(i+1,size(optPath_xy_smooth,1)),:);
    if cur_line(3) ~= next_line(3)
        optimal_xy_unique = [optimal_xy_unique;cur_line];
    end
end
optimal_xy_unique = [optimal_xy_unique;optPath_xy_smooth(end,:)];
path = optimal_xy_unique(:,1:2);
nodeIndex = optimal_xy_unique(:,3);
cur_pos = [];
prev_pos = [];
numpts = size(path,1);
for i = 1:numpts
    prev_pos = cur_pos;
    cur_pos = path(i,:);
    if i ~= numpts
        next_pos = path(i+1,:);
    end
    temp_cur = cur_pos;
    if ~isempty(prev_pos)
        plot_UGV_on_path(prev_pos(1),prev_pos(2),cur_pos(1),cur_pos(2))
    end
    if Nodes_TSP(round(optimal_xy_unique(i,3))).TID ~= -1
        plot_FOV(optimal_xy_unique(i,1),optimal_xy_unique(i,2),Nodes_TSP(round(optimal_xy_unique(i,3))).theta,true);
    end
        
    if isempty(prev_pos)
        sensor = directional_sensor(cur_pos',0,110*pi/180,50,100);
        draw_sensor(sensor,workspace);
        xlim([0,40])
        ylim([0,40])
    end
    %pause
    t_dir = atan2(next_pos(2) - cur_pos(2), next_pos(1) - cur_pos(1));
    sensor = directional_sensor(cur_pos',t_dir,110*pi/180,50,100);
    draw_sensor(sensor,workspace);
    hold on
    xlim([0,40])
    ylim([0,40])
    if i == 13
        sensor = directional_sensor(cur_pos',-pi/2,110*pi/180,50,100);
        draw_sensor(sensor,workspace);
    end
    xlim([0,40])
    ylim([0,40])
        
    pause
    disp('Here I pause')
end

