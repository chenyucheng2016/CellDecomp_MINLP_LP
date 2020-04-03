% plot C Target Grid
clear all
close all
figure
hold on



% A = CTarget();
% A = [0  1;0  0];
A = [1  1  1   1;0  0  0  0;1   0   1  0; 0  1  0   1];
x = [0  9.2];
y = [0  9.2];
% colormap(gray)
% im.AlphaData = .2;
% A = flipud(A);
% imagesc(x,y,A)
imagesc(A)

cmap = [.7 .7 .7 %// light gray
        1  1  1]; %// white
colormap(cmap)
% colorbar('Ytick',[.25 .75],'Yticklabel',[0 1]) 
% colorbar
set(gca,'YDir','normal')  % otherwise the y axis is flipped
% set(gca,'YDir','reverse');
xlim([0  9.2])
ylim([0  9.2])
axis equal

function A = CTarget()
n = 100;  % # of step of sampling
myStart = 0;
myEnd = 9.2;
step = (myEnd-myStart)/n;
xs = linspace(myStart,myEnd,n);
ys = linspace(myStart,myEnd,n);
A = zeros(n,n);

for i = 1:length(xs)
    for j = 1:length(ys)
        loc = [myStart+(i-1)*step+ step/2  myStart+(j-1)*step+step/2];
        
%         plot(loc(1),loc(2),'ko');
          A(i,j) = round(rand);
%         myRand = round(rand);
%         if myRand == 0   
%             A(i,j) = 1;  % random 0 or 1     
%         else
%             A(i,j) = 0; 
%         end
    end
end
end