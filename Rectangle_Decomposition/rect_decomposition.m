function decomposed_rect=rect_decomposition(workspace_rect,obstacle_rect)
% transfer more than one group bounding_layer1ob1{i}, i=1,...,n into one new group
% workspace_rect is a cell, workspace_rect{i} is a rectangle. Suppose every rectangle in this cell is disjoint.
% obstacle_rect is a cell, obstacle_rect{j} is a rectangle.
% This function is to decompose the space of Union{workspace_rect{i}}\ Union{obstacle_rect{j}}
% into a set of rectangles whose union is the space.
% to obtain the adjancyship of the decomposed rectangles, you can run the file rect_adj.m. 
% Say, the number of decomposed rectangles in decomposed_rect is N. 
% You build a N*N % matrix and use a double loop to obtain the adjanceyship of these N rectangle.

decomposed_rect={};

if length(obstacle_rect)<1
    decomposed_rect=bounded_target_inonelayer;
    return;
end
workspace_rect_back={};
obstacle_rect_back={};

j=0;
 %may first use new_group=mergeinonegroup(old_group)
for i=1:length(workspace_rect)
    if rect_area(workspace_rect{i})>0.005
       j=j+1;
       workspace_rect_back{j}=workspace_rect{i};
    end
end
j=0;
 %may first use new_group=mergeinonegroup(old_group)
for i=1:length(obstacle_rect)
    if rect_area(obstacle_rect{i})>0.005
       j=j+1;
       obstacle_rect_back{j}=obstacle_rect{i};
    end
end

change=1;
while change~=0 %& length(workspace_rect_back)>1
    if length(workspace_rect_back)==0
        decomposed_rect=workspace_rect_back;
        return;
    end
    for i=1:length(workspace_rect_back)
        for j=1:length(obstacle_rect_back)
            R1=workspace_rect_back{i};
            R2=obstacle_rect_back{j};
            [new_rect, change]=rect_diff(R1,R2);
            
                if change~=0
                    index_nochange=setdiff([1:length(workspace_rect_back)],[i]);
                    workspace_rect_back=[new_rect workspace_rect_back(index_nochange)];
                    break;
               end
         end
        if change~=0
            break;
        end
    end
end
decomposed_rect=workspace_rect_back;