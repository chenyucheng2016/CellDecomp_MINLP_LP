% Create some polar data to work with.
theta = linspace(0,2*pi,40);
rho = [5:.5:10]';
[rho,theta] = meshgrid(rho,theta);
% Map the polar coordinates to Cartesian coordinates.
[X,Y] = pol2cart(theta,rho);
Z = (sin(X)./X) + .05*sin(3*Y);
% Generate a black wire mesh plot.
hm = mesh(X,Y,Z);
set(hm,'FaceColor','none','EdgeColor','k')
hold on
ax = axis;
axis(ax)
% Project Data to the different planes.
h(1) = surf(X,Y,ax()+0*Z,Z); % X-Y Plane Projection
h(2) = surf(X,ax(4)+0*Y,Z);   % X-Z Plane Projection
h(3) = surf(ax(2)+0*X,Y,Z);   % Y-Z Plane Projection
set(h,'FaceColor','interp','EdgeColor','interp')
% Build a colormap that consists of three separate
% colormaps.
cmapX = bone(32);
cmapY = cool(32);
cmapZ = jet(32);
cmap = [cmapX;cmapY;cmapZ];
colormap(cmap)
% Map the CData of each surface plot to a contiguous, 
% nonoverlapping set of data.  Each CData must have
% the same range.
zmin = min(Z(:));
zmax = max(Z(:));
% CDX ranges from 1 to 32.
cdx = min(32,round(31*(Z-zmin)/(zmax-zmin))+1);
% CDY ranges from 33 to 64.
cdy = cdx+32;
% CDZ ranges from 65 to 96.
cdz = cdy+32;
% Update the CDatas.
set(h(1),'CData',cdx)
set(h(3),'CData',cdy)
set(h(2),'CData',cdz)
% Change CLim (Color Limits) so that it spans all the CDatas
caxis([min(cdx(:)) max(cdz(:))])

% % Define a colormap that uses the cool colormap and 
% % the gray colormap and assign it as the Figure's colormap.
% colormap([cool(64);gray(64)])
% % Generate some surface data.
% [X,Y,Z] = peaks(30);
% % Produce the two surface plots.
% h(1) = surf(X,Y,Z);
% hold on
% h(2) = pcolor(X,Y,Z);
% hold off
% % Move the pcolor to Z = -10.
% % The 0*Z is in the statement below to insure that the size
% % of the ZData does not change.
% set(h(2),'ZData',-10 + 0*Z)
% set(h(2),'FaceColor','interp','EdgeColor','interp')
% view(3)
% % Scale the CData (Color Data) of each plot so that the 
% % plots have contiguous, nonoverlapping values.  The range 
% % of each CData should be equal. Here the CDatas are mapped 
% % to integer values so that they are easier to manage; 
% % however, this is not necessary.
% % Initially, both CDatas are equal to Z.
% m = 64;  % 64-elements is each colormap
% cmin = min(Z(:));
% cmax = max(Z(:));
% % CData for surface
% C1 = min(m,round((m-1)*(Z-cmin)/(cmax-cmin))+1); 
% % CData for pcolor
% C2 = 64+C1;
% % Update the CDatas for each object.
% set(h(1),'CData',C1);
% set(h(2),'CData',C2);
% % Change the CLim property of axes so that it spans the 
% % CDatas of both objects.
% caxis([min(C1(:)) max(C2(:))])
% 
% % figure
% % %% Create two axes
% % ax1 = axes;
% % [x,y,z] = peaks;
% % surf(ax1,x,y,z)
% % view(2)
% % ax2 = axes;
% % scatter(ax2,randn(1,120),randn(1,120),50,randn(1,120),'filled')
% % %% Link them together
% % linkaxes([ax1,ax2])
% % %% Hide the top axes
% % ax2.Visible = 'off';
% % ax2.XTick = [];
% % ax2.YTick = [];
% % %% Give each one its own colormap
% % colormap(ax1,'hot')
% % colormap(ax2,'cool')
% % %% Then add colorbars and get everything lined up
% % set([ax1,ax2],'Position',[.17 .11 .685 .815]);
% % cb1 = colorbar(ax1,'Position',[.05 .11 .0675 .815]);
% % cb2 = colorbar(ax2,'Position',[.88 .11 .0675 .815]);
% 
% 
% % clear all
% % figure
% % 
% % 
% % hold on
% % 
% % [x,y,z] = peaks;
% % 
% % surf(x,y,z,'edgecolor','none');
% % hold on
% % grid on
% % 
% % 
% % 
% % [X,map] = imread('newWorld.png');
% % 
% % % 
% % % colormap(map);
% % 
% % x=xlim;
% % y=ylim;
% % 
% % surf([x;x],[y(1) y(1) ; y(2) y(2)],[-pi  -pi;-pi  -pi],...
% %     'facecolor','texturemap',...
% %     'cdata',X);
% % 
% % xlabel('x')
% % ylabel('y')
% % zlabel('z')
% % 
% % view(3)
% 
% 
% % figure
% % hold on
% % 
% % t = 0:pi/50:10*pi;
% % plot3(sin(t),cos(t),t);
% % 
% % grid on
% % 
% % load clown
% % colormap(map)
% % 
% % x=xlim
% % y=ylim
% % 
% % surf([x;x],[y(1) y(1) ; y(2) y(2)],zeros(2),...
% %     'facecolor','texturemap',...
% %     'cdata',X(end:-1:1,:))
% % 
% % xlabel('x')
% % ylabel('y')
% % zlabel('z')
% % 
% % view(3)
% 
% 
% 
% 
% 
% 
% 
% 
% 
% % % the data that you want to plot as a 3D surface.
% % [x,y,z] = peaks;
% %  
% % % get the corners of the domain in which the data occurs.
% % min_x = min(min(x));
% % min_y = min(min(y));
% % max_x = max(max(x));
% % max_y = max(max(y));
% %  
% % % the image data you want to show as a plane.
% % planeimg =flipud(imread('newWorld.png'));
% %  
% % % scale image between [0, 255] in order to use a custom color map for it.
% % minplaneimg = min(min(planeimg)); % find the minimum
% % scaledimg = (floor(((planeimg - minplaneimg) ./ ...
% %     (max(max(planeimg)) - minplaneimg)) * 255)); % perform scaling
% %  
% % % convert the image to a true color image with the jet colormap.
% % colorimg = ind2rgb(scaledimg,jet(256));
% %  
% % % set hold on so we can show multiple plots / surfs in the figure.
% % figure; hold on;
% %  
% % % do a normal surface plot.
% % surf(x,y,z,'edgecolor','none');
% %  
% %  
% % % desired z position of the image plane.
% % imgzposition = -10;
% %  
% % % plot the image plane using surf.
% % surf([min_x max_x],[min_y max_y],repmat(imgzposition, [2 2]),...
% %     colorimg,'facecolor','texture')
% %  
% % % set the view.
% % view(45,30);
% %  
% % % label the axes
% % xlabel('x');
% % ylabel('y');
% % zlabel('z');
% % % 
% % % % the data that you want to plot as a 3D surface.
% % % [x,y,z] = peaks;
% % %  
% % % 
% % %  
% % % % the image data you want to show as a plane.
% % % planeimg =flipud(imread('newWorld.png'));
% % %  
% % %  
% % % % set hold on so we can show multiple plots / surfs in the figure.
% % % figure; hold on;
% % %  
% % % % do a normal surface plot.
% % % surf(x,y,z,'edgecolor','none');
% % %  
% % % % desired z position of the image plane.
% % % imgzposition = -pi;
% % %  
% % % % plot the image plane using surf.
% % % % surf([min_x max_x],[min_y max_y],repmat(imgzposition, [2 2]),...
% % % %     planeimg,'facecolor','texture')
% % % surf([0  9.2],[0  9.2],repmat(imgzposition, [2 2]),...
% % %     planeimg,'facecolor','texture')
% % %  
% % % % set the view.
% % % view(45,30);
% % %  
% % % 
% % % % figure;
% % % % Im=flipud(imread('newWorld.png'));
% % % % 
% % % % %// Dummy surface to plot.
% % % % Z = peaks(9);
% % % % 
% % % % %// Prepare image position
% % % % shift = pi;
% % % % xIm=zeros(9)-shift;
% % % % 
% % % % hold on
% % % % surface(xIm,Im,'FaceColor','texturemap','EdgeColor','none','CDataMapping','direct')
% % % % % surface(Z,'FaceAlpha',0.8,'LineStyle','none','FaceColor','interp');
% % % % axis([0 9 0 9])
% % % % axis on
% % % % % view(-35,45)
% % % % 
% % % % box on
% % % % rotate3d on
% % % % axis equal
% % % 
