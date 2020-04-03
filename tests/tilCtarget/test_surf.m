% clear all
% load ('3dCTarget_v1');
% 
% 
% m = length(B);
% ts = linspace(0,2*pi,m+1); % theta 
% figure;hold on
% 
% surf(B{1});
% hold on
% surf(B{2});
% surf(B{3});
% surf(B{10});
% surf(B{5});
% for i =1: length(B)
% %     height = ts(i);
%     surf(B{i});
% 
% end

% 
% figure;hold on
% [xx yy] = meshgrid(1:10);
% % colormap([1 0 0;0 0 1]) %red and blue
% surf(xx,yy,rand(10),ones(10)); %first color (red)
% surf(xx,yy,rand(10)+10,ones(10)+1); %second color(blue)
% view(17,22)

figure
% [X,Y,Z] = cylinder(0.01);
% X = X+3;
% Y = Y+3;
% Z = Z*2*pi;
% surf(X,Y,Z)
contour(B3d{45})