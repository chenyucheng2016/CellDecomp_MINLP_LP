function tot_cost = compuetDist(Nodes_TSP,nodeSequence)
tot_cost = 0;
len = length(nodeSequence);
for i = 1:len-1
    startNode = nodeSequence(i);
    endNode = nodeSequence(i+1);
    start_vert = Nodes_TSP(startNode).vertices{1};
    end_vert = Nodes_TSP(endNode).vertices{1};
    x_center_start = (max(start_vert(:,1)) + min(start_vert(:,1)))/2;
    y_center_start = (max(start_vert(:,2)) + min(start_vert(:,2)))/2;
    x_center_end = (max(end_vert(:,1)) + min(end_vert(:,1)))/2;
    y_center_end = (max(end_vert(:,2)) + min(end_vert(:,2)))/2;
    tot_cost = tot_cost + sqrt((x_center_start - x_center_end)^2 + (y_center_start - y_center_end)^2);
end
end