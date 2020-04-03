function adjacentNodes = findAdjacentNods(conn_pair,node_ID)
num = size(conn_pair,1);
adjacentNodes = [];
for i = 1:num
    edge = conn_pair(i,:);
    if edge(1) == node_ID
        adjacentNodes = [adjacentNodes,edge(2)];
    end
    if edge(2) == node_ID
        adjacentNodes = [adjacentNodes,edge(1)];
    end
end
adjacentNodes  = unique(adjacentNodes);
end