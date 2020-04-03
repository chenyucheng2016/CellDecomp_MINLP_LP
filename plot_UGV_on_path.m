function plot_UGV_on_path(x1,y1,x2,y2)
global ugv_h  ugv_w
line_length = distPts(x1,y1,x2,y2);

gap = 0.5;

num_ugv = floor((line_length)/(ugv_w + gap));
if num_ugv < 1
    num_ugv = 1;
end

xs = linspace(x1 ,x2 ,num_ugv);
ys = linspace(y1 ,y2 ,num_ugv);

direction_rad = deg2rad(get_target_direction([x1 y1], [x2 y2]));

if num_ugv == 1
    DrawRectangle([xs(1),ys(1),ugv_w,ugv_h,direction_rad]);
    %DrawEquilTriangle([xs(1) ys(1)], direction_rad + deg2rad(90));
else
    for count_ugv = 1: num_ugv-1
        cur_x = xs(count_ugv); cur_y = ys(count_ugv);
        DrawRectangle([cur_x,cur_y,ugv_w,ugv_h,direction_rad]);   %param = [a, b, w, h, theta], a and b are center
        if count_ugv == num_ugv-1
            %DrawEquilTriangle([cur_x cur_y], direction_rad + deg2rad(90));
        end
    end
end


function DrawEquilTriangle(pos, theta)
    scale = 0.4;
    verts = [0 0; [1 1.73205]*scale; [-1 1.73205]*scale];
    
    % rotation matrix
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    verts = R*(verts');
    verts = verts' + pos;

    x = verts(:,1);
    y = verts(:,2);
    
    patch(x, y, 'g','FaceAlpha',.2);
end

% given the target pos and ms pos, return the target direction in degrees
function dir = get_target_direction(pos1, pos2)
    % the right direction
    x1 = 1; y1 = 0; 
    % the vector from ms to target
    x2 = pos1(1) - pos2(1);
    y2 = pos1(2) - pos2(2);
    % angle between 2 vector
    dir = angle_between(x1,y1,x2,y2);
end

% angle between 2 vector
function  a = angle_between(x1,y1,x2,y2)
    a = atan2d(x1*y2-y1*x2,x1*x2+y1*y2);
end
end