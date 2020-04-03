% TESTED  ifLineCross

clear all
close all



% line
robotx = 0; roboty = 0; targetx = 9; targety = 9;
xs = [targetx robotx];
ys = [targety  roboty];
plot(xs,ys);
hold on
axis equal



% disp( pointOnLine(9,9,xs,ys));
disp( ifLineCross(xs,ys))

function flag = ifLineCross(xs,ys)
% circle 
cx = 6; cy = 6; r = 2;
c = [cx   cy];
pos = [c-r 2*r 2*r];
rectangle('Position',pos,'Curvature',[1 1],'EdgeColor','k')

[k,b] = slopeIntercept(xs,ys);
[xout,yout] = linecirc(k,b,cx,cy,r);
plot(xout,yout,'ro');


if isempty(xout)
    flag = false;
else
    if cutLine(xout,yout,xs,ys)
        flag = false;  % line NOT cross
    else
        flag = true;
    end
end
end

% check if a point is on the line segment given by lineX lineY
function tf = pointOnLine(pointX,pointY,lineX,lineY)
tol = 0.0001; % tolerance
distance = dist(pointX,pointY, lineX(1),lineY(1)) + dist(pointX,pointY, lineX(2),lineY(2)) - dist(lineX(2),lineY(2), lineX(1),lineY(1));
tf = (distance<tol);
end

% distance between two points
function d = dist(x1,y1,x2,y2)
d = sqrt((y2-y1)^2+(x2-x1)^2);
end


% flag ture, can see target
function flag = cutLine(xout,yout,lineX,lineY)
flag = true;
for i = 1:length(xout)
    % if any intersection on the line
    if  pointOnLine( xout(i),yout(i),lineX,lineY)
        flag = false;
        return
    end
end
end




% flag ture, can see target
function flag = reachTargetDist(xout,yout,robotx,roboty,targetx,targety)
flag = true;
for i = 1:length(xout)
    x = xout(i); y = yout(i); % intersection point with circle
    % if intersect with obstacle before reaching target, fail to reach
    if (x-robotx)^2+(y-roboty)^2 < (targetx-robotx)^2+(targety-roboty)^2
        flag = false;
        return
    end
end
end


% convert from two points to slope and intersect of line
function [k,b] = slopeIntercept(xs,ys)
k = (ys(2)-ys(1))/(xs(2)-xs(1));
b = ys(1)-k*xs(1);
end