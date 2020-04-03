x1 = 2; y1 = 1;
x2 = 5; y2 = 9;
% x2 = 1; y2 = 9;
figure;
plot([x1  x2],[y1  y2],'k-');
xlim([0  9.2]);  ylim([0  9.2]);
axis equal

hold on
xq = (x1 + x2)/2;   yq = (y1 + y2)/2; 
plot(xq,yq,'r*');
ang = atan2(x2-x1,y1-y2);
r = 3;
x3 = xq+cos(ang)*r;
y3 = yq+sin(ang)*r;
plot(x3,y3,'r*');
x4 = xq-cos(ang)*r;
y4 = yq-sin(ang)*r;
plot(x4,y4,'r*');
% testK = (y1-y2)/(x2-x1);

% syms x3  y3  k  r  xq  yq
% S = solve((y3-yq)==k*(x3-xq),(y3-yq)^2+(x3-xq)^2==r^2);
