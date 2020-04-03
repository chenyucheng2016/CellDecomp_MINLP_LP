%plotFinal Map
optPath_xy_smooth_mod = optPath_xy_smooth;
optPath_xy_smooth_mod(33,2) = optPath_xy_smooth(33,2) + 1.2;
optPath_xy_smooth_mod(64,1) = optPath_xy_smooth(64,1) - 0.8;
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
line(optPath_xy_smooth_mod(:,1),optPath_xy_smooth_mod(:,2),'Color','red')
toc
for i = 1:size(optPath_xy_smooth,1)
    hold on
    if Nodes_TSP(round(optPath_xy_smooth(i,3))).TID == -1
       plot_FOV(optPath_xy_smooth_mod(i,1),optPath_xy_smooth_mod(i,2),Nodes_TSP(round(optPath_xy_smooth(i,3))).theta,false);
    else
        plot_FOV(optPath_xy_smooth_mod(i,1),optPath_xy_smooth_mod(i,2),Nodes_TSP(round(optPath_xy_smooth(i,3))).theta,true);
    end 
end
axis equal
xlim([0,40])
ylim([0,40])
