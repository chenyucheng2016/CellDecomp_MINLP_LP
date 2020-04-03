global mapLimit   % setting for canvas size 
mapLimit = [0  9.2];
global  A2d A3d B3d  B2d
A3d = load('A3d_300x60'); A3d = A3d.A3d;

 A2d = A3d(:,:,1); % 2d projection
%  i = 300; j = 1;
% [x,y] = grid2xy(i,j)
x = 2.185;  y = 4.123;
[i,j] = xy2grid(x,y)

function [i,j] = xy2grid(x,y)
global A2d mapLimit 
Asize = size(A2d); n = Asize(1);% # of step of sampling
myStart = mapLimit(1);
myEnd = mapLimit(2);

step = (myEnd-myStart)/n;

i = floor((x-myStart)/step)+1;
j = floor((y-myStart)/step)+1;
if i > 300
    i = 300;
end
if j > 300
    j = 300;
end
end

function [x,y] = grid2xy(i,j)
global mapLimit B3d A2d
 n = length(A2d);% m = length(B3d{1});
myStart = mapLimit(1); myEnd = mapLimit(2);
step = (myEnd-myStart)/(n-1);
x = myStart+(i-1)*step;
y = myStart+(j-1)*step;

end