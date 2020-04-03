x1 = 1; y1 = 1; r1 = 1; x2 = 1; y2 = 1; r2 = 0.5;
[xout,yout] = circcirc(x1,y1,r1,x2,y2,r2);
tf = true; % circle conflict
if isempty(xout) && dist1(x1,y1,x2,y2)>= max(r1,r2)
    tf = false;
end
disp(tf)



figure;
r = r1;
%// center
c = [x1   y1];
pos = [c-r 2*r 2*r];
rectangle('Position',pos,'Curvature',[1 1],'EdgeColor','k');
hold on
axis equal
r = r2;
%// center
c = [x2   y2];
pos = [c-r 2*r 2*r];
rectangle('Position',pos,'Curvature',[1 1],'EdgeColor','k');

% distance between two points
function d = dist1(x1,y1,x2,y2)
d = sqrt((y2-y1)^2+(x2-x1)^2);
end
