

clear all
close all
figure;

r = 1;

plotCircle3d(r,1,1)
function plotCircle3d(r,cX,cY)
theta = 0:0.01:2*pi;
h = 0.01;
x = r*cos(theta)+cX;
y = r*sin(theta)+cY;

y(end) = cY;

z1 = -pi;



patch(x,y,z1*ones(size(x)),'k');
alpha(0.3);
end

% set(gca,'NextPlot','Add');

% patch(x,y,z2*ones(size(x)),'b');

% surf([x;x],[y;y],[z1*ones(size(x));z2*ones(size(x))],'parent',gca)