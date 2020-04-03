close all
% Define and fill a rectangular area in the plane
xlimit = [3 13];
ylimit = [2  8];
xbox = xlimit([1 1 2 2 1]);
ybox = ylimit([1 2 2 1 1]);
mapshow(xbox,ybox,'DisplayType','polygon','LineStyle','none')
 
% Define and display a two-part polyline
x = [0 8];
y = [4 6];
mapshow(x,y,'Marker','+')

% Intersect the polyline with the rectangle
[xi, yi] = polyxpoly(x, y, xbox, ybox);

if isempty(xi)
    disp('no such thing');
else
    mapshow(xi,yi,'DisplayType','point','Marker','o');
end


% for circle
% [xout,yout] = linecirc(slope,intercpt,centerx,centery,radius)