function is_C_adj=rect_C_adj(R1, R2, theta)
% returns if two rectangles are adjacent in the theta direction
is_adj = rect_adj(R1, R2);

min_x_a=min(R1(1,:));
max_x_a=max(R1(1,:));
min_y_a=min(R1(2,:));
max_y_a=max(R1(2,:));
min_x_b=min(R2(1,:));
max_x_b=max(R2(1,:));
min_y_b=min(R2(2,:));
max_y_b=max(R2(2,:));

is_C_adj = 0;
if abs(theta-2*pi)<1e-3 || abs(theta-pi)<1e-3
    if is_adj == 1 && (min_x_a == max_x_b || max_x_a == min_x_b)
        is_C_adj = 1;
    end
end
if abs(theta - 1.5*pi)<1e-3 || abs(theta - 0.5*pi)<1e-3
    if is_adj == 1 && (min_y_a == max_y_b || max_y_a == min_y_b)
        is_C_adj = 1;
    end
end


end

