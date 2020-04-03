% test inpolygon
close all; clear all;
figure



 xs =  [0.3667    0.3667    2.1667    2.1667 ];
 ys  =  [0.3225    1.1225    1.1225    0.3225];
 
plot(xs,ys) % polygon
hold on
axis equal
xlim = [0   9.2]; ylim = [0  9.2];

xq = 1; yq = 0.5;
in = inpolygon(xq,yq,xs,ys)
plot(xq,yq,'r*');