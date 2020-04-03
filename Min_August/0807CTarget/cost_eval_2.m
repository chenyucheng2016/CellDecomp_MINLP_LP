function cost = cost_eval_2(w_trans,Nodes_path,theta_start)
% Given a path defined by trip_raw, compute the cost with w_trans

cost = 0;
trans_tot = 0;
r_tot = 0;

w_rotat = 1 - w_trans;

v1 = Nodes_path{1}.vertices{1};
v2 = Nodes_path{2}.vertices{1};
x1 = mean(v1(:,1));
y1 = mean(v1(:,2));
x2 = mean(v2(:,1));
y2 = mean(v2(:,2));
xy1 = [x1, y1, 0];
xy2 = [x2, y2, 0];
u = xy2 - xy1;
v = [cos(theta_start), sin(theta_start), 0];
cost = cost + w_rotat * atan2(norm(cross(u,v)),dot(u,v));
trans_tot = trans_tot+0;
r_tot = r_tot + atan2(norm(cross(u,v)),dot(u,v));

for i = 1:length(Nodes_path)-2

    v1 = Nodes_path{i}.vertices{1};
    v2 = Nodes_path{i+1}.vertices{1};
    v3 = Nodes_path{i+2}.vertices{1};
    
    x1 = mean(v1(:,1));
    y1 = mean(v1(:,2));
    x2 = mean(v2(:,1));
    y2 = mean(v2(:,2));
    x3 = mean(v3(:,1));
    y3 = mean(v3(:,2));
    xy1 = [x1, y1, 0];
    xy2 = [x2, y2, 0];
    xy3 = [x3, y3, 0];
    cost = cost + w_trans * sqrt(sum((xy2 - xy1).^2));
    u = xy2 - xy1;
    v = xy3 - xy2;
    cost = cost + w_rotat * atan2(norm(cross(u,v)),dot(u,v));
    
    trans_tot = trans_tot+sqrt(sum((xy2 - xy1).^2));
    r_tot = r_tot + atan2(norm(cross(u,v)),dot(u,v));

end

v1 = Nodes_path{i+1}.vertices{1};
v2 = Nodes_path{i+2}.vertices{1};
x1 = mean(v1(:,1));
y1 = mean(v1(:,2));
x2 = mean(v2(:,1));
y2 = mean(v2(:,2));

xy1 = [x1, y1];
xy2 = [x2, y2];
cost = cost + w_trans * sqrt(sum((xy2 - xy1).^2));
trans_tot = trans_tot+sqrt(sum((xy2 - xy1).^2));

end


