% figure
% 
% targets(4).loc = [7.25,7.2];
% inds = [28,14,29,6,26,13,24,1];
% xt4 = targets(4).loc;
% for i=1:length(inds)
%     xt = targets(inds(i)).loc;
%     plot([xt4(1),xt(1)],[xt4(2),xt(2)],'r-')
%     hold on
%     x1 = xt4(1);
%     y1 = xt4(2);
%     x2 = xt(1);
%     y2 = xt(2);
%     angle = atan2(y2-y1, x2-x1);
%     % plot intermediate robots
%     num_plot = floor(sqrt(sum(([x1,y1]-[x2,y2]).^2))/plot_grid);
%     t = inds(i);
%     if t==1
%         upper = 2;
%     end
%     if t==13
%         upper = 4;
%     end
%     if t==6
%         upper = 3;
%     end
%     if t==29
%         upper = 2;
%     end
%     if t==14
%         upper = 3;
%     end
%     if t==26
%         upper = 2;
%     end
%     if t==28
%         upper = 1;
%     end
%     for m = upper:upper
%         plot_FOV(x1+plot_grid*m*cos(angle), y1+plot_grid*m*sin(angle), angle, false);
%         hold on
%     end
%     hold on
% end
% 
% 
% for i=1:length(ind_t)
%     xt = targets(ind_t(i)).loc;
% %     if any(xt > w_xmax)
% %         continue
% %     end
%     
% %     plot(xt(1),xt(2),'ko') 
%     plot(xt(1),xt(2),'.r', 'MarkerSize',20)
%     if i~=30 && i ~= 19
%         text(xt(1),xt(2)-0.1,num2str(i))
%     end
%     hold on
% end
% 
% for i=1:size(walls,2)
%     v = walls(i).vertices;
%     x = round(v(1,:),1);
%     y = round(v(2,:),1);  
%     k = convhull(x,y);
%     fill(x(k),y(k),'k')
% %     text(x,y-0.1,num2str(i))
%     hold on
% end
% 
% for i=1:size(recFur,2)
%     v = recFur(i).vertices;
%     x = round(v(1,:),1);
%     y = round(v(2,:),1);
%     k = convhull(x,y);
%     fill(x(k),y(k),'k')
% %     text(x,y-0.1,num2str(i))
%     hold on
% end
% axis equal
% xlim([4.5,9.2])
% ylim([4.5,9.2])
% set(gca,'XTick',[4 : 1: 9]);
% set(gca,'YTick',[4 : 1: 9]);

[cost,trans_tot,r_tot] = cost_eval_5(w_trans,Nodes_path_all_smooth,pi/2)
