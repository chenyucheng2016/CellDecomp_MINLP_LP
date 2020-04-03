global mapLimit B2d
i = 1; j = 1;
B2d = zeros(10,10);
B2d(i,j) = 1;


mapLimit = [0  9.2];
figure
hold on
plotCTarget(B2d);
axis equal
[x,y] = grid2xy(i,j);
plot(x,y,'r*')

function [x,y] = grid2xy(i,j)
global mapLimit B2d
n = length(B2d); 
myStart = mapLimit(1); myEnd = mapLimit(2);
step = (myEnd-myStart)/(n-1);
x = myStart+(j-1)*step;
y = myStart+(i-1)*step;

end

function plotCTarget(B)
global mapLimit
x = mapLimit;y = mapLimit;


% colormap(gray)
% im.AlphaData = .2;
 imagesc(x,y,B)

cmap = [.5 .5 .5 %// light gray
        1  1  1]; %// white
colormap(cmap);
% colorbar('Ytick',[.25 .75],'Yticklabel',[0 1]) 
% colorbar
set(gca,'YDir','normal')  % otherwise the y axis is flipped
end