clear all
close all
clc
figure
B = load('B3d_300x60'); B = B.B3d;
h = B{1};
global mapLimit
mapLimit = [0  9.2];
xs = linspace(mapLimit(1),mapLimit(2),length(h));
%org = [];
ax1 = axes;


for i =1:length(B)
    h(i) = surf(ax1,xs,xs,B{i});hold on

set(h(i), 'FaceAlpha',0.7, 'EdgeAlpha', 0.3,'EdgeColor','none');
% get(h(i),'FaceColor');

%     s.EdgeColor = 'none';'FaceColor',[rand rand rand], 
%     org=vertcat(colormap,org);
end
% ax1 = h;
%colormap(ax1,org);

% hold on
 [X,map] = imread('newWorld.png');
% % [X, map] = rgb2ind(imread('newWorld.png'),64);
 x=mapLimit; y=mapLimit;
% % % 
 ax2 = axes;
h = surf(ax2,[x;x],[y(1) y(1) ; y(2) y(2)], zeros(2)-pi,...
    'facecolor','texturemap',...
   'cdata',X);hold on
set(h, 'FaceAlpha',0.6);
zlim([-pi pi])

linkaxes([ax2,ax1])
%% Hide the top axes
ax2.Visible = 'off';
ax2.XTick = [];
ax2.YTick = [];
%% Give each one its own colormap
%colormap(ax1,'hot')
%colormap(ax2,'cool')
%% Then add colorbars and get everything lined up
%set([ax1,ax2],'Position',[.17 .11 .685 .815]);
%cb1 = colorbar(ax1,'Position',[.05 .11 .0675 .815]);
%cb2 = colorbar(ax2,'Position',[.88 .11 .0675 .815]);



% linkaxes([ax1,ax2]);
% % surf([x;x],[y(1) y(1) ; y(2) y(2)],-pi*ones(2),'facecolor','texturemap','cdata',map);
% % org=vertcat(org,map);
% % colormap(org);
% % plotAll(all);
% 
% % %plot settings
% % axis equal
% % title('3D C-target')
% xlabel('x(m)')
% ylabel('y(m)')
% zlabel('\theta (rad)')
% colorbar
% colorbar('Ticks',[-pi -pi/2 0 pi/2 pi],...
%          'TickLabels',{'-\pi','-\pi/2','0','\pi/2','pi',})
% zticks([-pi -pi/2 0 pi/2 pi])
% zticklabels({'-\pi','-\pi/2','0','\pi/2','pi',})
% set(gca,'FontSize',15)
% grid on
% hold on
% 




% % Create some polar data to work with.
% theta = linspace(0,2*pi,40);
% rho = [5:.5:10]';
% [rho,theta] = meshgrid(rho,theta);
% % Map the polar coordinates to Cartesian coordinates.
% [X,Y] = pol2cart(theta,rho);
% Z = (sin(X)./X) + .05*sin(3*Y);
% % Generate a black wire mesh plot.
% hm = mesh(X,Y,Z);
% set(hm,'FaceColor','none','EdgeColor','k')
% hold on
% ax = axis;
% axis(ax)
% % Project Data to the different planes.
% h(1) = surf(X,Y,ax(5)+0*Z,Z); % X-Y Plane Projection
% h(2) = surf(X,ax(4)+0*Y,Z);   % X-Z Plane Projection
% h(3) = surf(ax(2)+0*X,Y,Z);   % Y-Z Plane Projection
% set(h,'FaceColor','interp','EdgeColor','interp')
% % Build a colormap that consists of three separate
% % colormaps.
% cmapX = bone(32);
% cmapY = cool(32);
% cmapZ = jet(32);
% cmap = [cmapX;cmapY;cmapZ];
% colormap(cmap)
% % Map the CData of each surface plot to a contiguous, 
% % nonoverlapping set of data.  Each CData must have
% % the same range.
% zmin = min(Z(:));
% zmax = max(Z(:));
% % CDX ranges from 1 to 32.
% cdx = min(32,round(31*(Z-zmin)/(zmax-zmin))+1);
% % CDY ranges from 33 to 64.
% cdy = cdx+32;
% % CDZ ranges from 65 to 96.
% cdz = cdy+32;
% % Update the CDatas.
% set(h(1),'CData',cdx)
% set(h(3),'CData',cdy)
% set(h(2),'CData',cdz)
% % Change CLim (Color Limits) so that it spans all the CDatas
% caxis([min(cdx(:)) max(cdz(:))])

% figure
% view(3)
% %% Create two axes
% 
% 
% 
% [X,map] = imread('newWorld.png');
% x=xlim;
% y=ylim;
% 
% h(1) = surf([x;x],[y(1) y(1) ; y(2) y(2)],[-pi  -pi;-pi  -pi],...
%     'facecolor','texturemap',...
%     'cdata',X);
% hold on
% 
% 
% [x,y,z] = peaks;
% h(2) = surf(x,y,z);
% set(h(2),'ZData',-10 + 0*Z)
% set(h(2),'FaceColor','interp','EdgeColor','interp')
% 
% m = 64;  % 64-elements is each colormap
% cmin = min(z(:));
% cmax = max(z(:));
% % CData for surface
% C1 = min(m,round((m-1)*(z-cmin)/(cmax-cmin))+1); 
% % CData for pcolor
% C2 = 64+C1;
% % Update the CDatas for each object.
% 
% set(h(1),'CData',C1);
% set(h(2),'CData',C2);
% % %% Then add colorbars and get everything lined up
% % set([ax1,ax2],'Position',[.17 .11 .685 .815]);
% % cb1 = colorbar(ax1,'Position',[.05 .11 .0675 .815]);
% % cb2 = colorbar(ax2,'Position',[.88 .11 .0675 .815]);