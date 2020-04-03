%plot Big Workspace
%clc,clear,close all
%load('CDC_BIGWORKSPACE.mat')
close all
figure
for i=1:length(ind_t)
    xt = targets(ind_t(i)).loc;
    init_feature = init_features(ind_t(i));
    init_feature_index = encodeFeatures(init_feature);
    feature_level = computeFeatureLevel(init_feature_index);
    if feature_level == 0
        targ_label1 = plot(xt(1),xt(2),'or', 'MarkerSize',8,'LineWidth',2.5);
    elseif feature_level == 1
        targ_label2 = plot(xt(1),xt(2),'sr', 'MarkerSize',8,'LineWidth',2.5);
    elseif feature_level == 2
        targ_label3 = plot(xt(1),xt(2),'dr', 'MarkerSize',6,'LineWidth',2.5);
    else
        targ_label4 = plot(xt(1),xt(2),'+r', 'MarkerSize',8,'LineWidth',2.5);
    end
    text(xt(1),xt(2)-0.1,num2str(i))
    hold on
end
for i=1:size(recFur,2)
    v = recFur(i).vertices;
    x = round(v(1,:),1);
    y = round(v(2,:),1);
    
    k = convhull(x,y);
    obs_label = fill(x(k),y(k),'k');
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
line_label = line(optPath_xy_smooth(:,1),optPath_xy_smooth(:,2),'Color','red')
plotted = [];
for i = 1:size(optPath_xy_smooth,1)
    tar = Nodes_TSP(round(optPath_xy_smooth(i,3))).TID;
    hold on
    if tar == -1
       [~,~,~] = plot_FOV(optPath_xy_smooth(i,1),optPath_xy_smooth(i,2),Nodes_TSP(round(optPath_xy_smooth(i,3))).theta,false,-1);
    else
       if ~ismember(tar,plotted)
        tar_Measure = tarsMeasure(tar);
        if tar_Measure == 0
           [~,~,FOV1] = plot_FOV(optPath_xy_smooth(i,1),optPath_xy_smooth(i,2),Nodes_TSP(round(optPath_xy_smooth(i,3))).theta,true,tar_Measure+1);
        elseif tar_Measure == 1
           [~,~,FOV2] = plot_FOV(optPath_xy_smooth(i,1),optPath_xy_smooth(i,2),Nodes_TSP(round(optPath_xy_smooth(i,3))).theta,true,tar_Measure+1);
        elseif tar_Measure == 2
            [~,~,FOV3] = plot_FOV(optPath_xy_smooth(i,1),optPath_xy_smooth(i,2),Nodes_TSP(round(optPath_xy_smooth(i,3))).theta,true,tar_Measure+1);
        elseif tar_Measure == 3
            [~,~,FOV4] = plot_FOV(optPath_xy_smooth(i,1),optPath_xy_smooth(i,2),Nodes_TSP(round(optPath_xy_smooth(i,3))).theta,true,tar_Measure+1);
        end
            
       end
       plotted = [tar,plotted];
    end
end
drawnow
xlim([0.5,7])
ylim([0,7.5])
xticks(0:1:7)
%lgd = legend([obs_label,targ_label1,targ_label2,targ_label3,targ_label4,FOV1,FOV2,FOV3,FOV4,line_label],{'','','','','','','','','',''})
%lgd.FontSize = 10