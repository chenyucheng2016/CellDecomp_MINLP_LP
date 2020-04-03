% test PolyConflict
figure;
% x1 = [1  1  4  4];
% y1 = [1  2  2  1];
% x2 = [2  3  5  4];
% y2 = [3  3  1  1];

x1 = [1     2     2     1   1];
y1 = [1     1     2     2   1];
x2 = [0.9296    1.0296    1.0296    0.9296   0.9296];
y2 = [1.9580    1.9580    1.8380    1.8380    1.9580];
plot([x1(1,:)  x1(1)], [y1(1,:)  y1(1)])
hold on
plot([x2(1,:)  x2(1)], [y2(1,:)  y2(1)])
axis equal
% [xi,yi] = polyxpoly(x1,y1,x2,y2)

[xi,yi] = polyxpoly(x1,y1,x2,y2)