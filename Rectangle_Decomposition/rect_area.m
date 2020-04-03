function area=rect_area(R)
area=[];
min_x_a=min(R(1,:));
max_x_a=max(R(1,:));
min_y_a=min(R(2,:));
max_y_a=max(R(2,:));
area=(max_x_a-min_x_a)*(max_y_a-min_y_a);