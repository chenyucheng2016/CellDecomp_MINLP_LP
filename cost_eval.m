function cost = cost_eval(w_trans,trip_raw,Nodes_TSP,theta_start)
% Given a path defined by trip_raw, compute the cost with w_trans

cost = 0;

w_rotat = 1 - w_trans;

n1 = trip_raw(1);
n2 = trip_raw(2);
v1 = Nodes_TSP(n1).vertices{1};
v2 = Nodes_TSP(n2).vertices{1};
x1 = mean(v1(:,1));
y1 = mean(v1(:,2));
x2 = mean(v2(:,1));
y2 = mean(v2(:,2));
xy1 = [x1, y1, 0];
xy2 = [x2, y2, 0];
u = xy2 - xy1;
v = [cos(theta_start), sin(theta_start), 0];
cost = cost + w_rotat * atan2(norm(cross(u,v)),dot(u,v));

for i = 1:length(trip_raw)-2
    n1 = trip_raw(i);
    n2 = trip_raw(i+1);
    n3 = trip_raw(i+2);
    if n1 > length(Nodes_TSP) || n2 > length(Nodes_TSP)
        continue
    end
    contact = [];
    v1 = Nodes_TSP(n1).vertices{1};
    v2 = Nodes_TSP(n2).vertices{1};
    v3 = Nodes_TSP(n3).vertices{1};
    
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
end

if isempty(i)
   i = 0; 
end

v1 = Nodes_TSP(trip_raw(i+1)).vertices{1};
v2 = Nodes_TSP(trip_raw(i+2)).vertices{1};
x1 = mean(v1(:,1));
y1 = mean(v1(:,2));
x2 = mean(v2(:,1));
y2 = mean(v2(:,2));

xy1 = [x1, y1];
xy2 = [x2, y2];
cost = cost + w_trans * sqrt(sum((xy2 - xy1).^2));


end

