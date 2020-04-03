% draw fan shape
close all

direction = -pi-(pi/4)/2;  % direction

drawFan([0  0],3,pi/4,direction);
axis equal
function drawFan(startLoc,r,theta,direction)
x0 = startLoc(1);
y0 = startLoc(2);
direction2 = direction + theta;
t = linspace(direction,direction2);
x = x0 + r*cos(t);
y = y0 + r*sin(t);
plot([x0,x,x0],[y0,y,y0],'k-')
end