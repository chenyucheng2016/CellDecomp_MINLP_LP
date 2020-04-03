classdef recObs < obstacles
    properties
        vertices
    end
    methods
         % constructor
        function recObj = recObs(v,t)
            if nargin > 1
                super_arg{1} = t;
            else
                super_arg{1} = NaN;
            end
            recObj@obstacles(super_arg{:});
            if nargin > 1
                recObj.vertices = v;
            else
                recObj.vertices = NaN;
            end
%             if nargin > 1  % allows calls of constructor with no arguments, e.g. a(1,7) = points(7), 1-6 has not arguments
%                 obj@obstacles(t);
%                 obj.vertices = v;
%             else
%                 obj@obstacles();
%                 obj.vertices = NaN;
%             end
        end
        
        
         % draws a hollow countour according to coordinate arrays xs, ys. color is
        % optinonal, default is random
        function polyEdge(xs,ys,varargin)
               plot([xs(1,:)  xs(1)], [ys(1,:)  ys(1)],varargin{:});
        end
        
        
        % draws a single obstacle object, color (e.g.'k') and hollow(true) is optional.
        function plotObs(obs,edgeColor, hollow)
            if nargin == 1
                plot(obs.vertices(1,:), obs.vertices(2,:));
                fill(obs.vertices(1,:), obs.vertices(2,:));
            elseif nargin == 2
                plot(obs.vertices(1,:), obs.vertices(2,:),edgeColor);
                fill(obs.vertices(1,:), obs.vertices(2,:),edgeColor);
            elseif  nargin == 3
                if hollow == true
                    polyEdge(obs.vertices(1,:), obs.vertices(2,:),edgeColor);
                end
            end
        end
        
        
        % determines if a line cross this obstacle
        function tf = ifLineCross(obs,lineX,lineY,varargin)  %x = [0 10]; y = [4 0];
            
            xs = obs.vertices(1,:);
            ys = obs.vertices(2,:);
            
            [xi, yi] = polyxpoly(lineX, lineY, xs, ys);
            if isempty(xi)
                tf = false;
            else
                tf = true;
               
            end
           
            if ~isempty(varargin)
                 mapshow(lineX,lineY,'Marker','+')
                mapshow(xs,ys,'DisplayType','polygon','LineStyle','none');
                if ~isempty(xi)
                     mapshow(xi,yi,'DisplayType','point','Marker','o');
                end
            end
 
        end
        
        % determine if a point is in this obstacle
        function tf = ptInObs(obs,ptX,ptY)
            tf = inpolygon(ptX,ptY,obs.vertices(1,:),obs.vertices(2,:));
        end
        
        % determine if a polygon instersects with this obstacle
        function tf = polyConflict(obs,otherX,otherY)
           [xi,yi] = polyxpoly([obs.vertices(1,:)  obs.vertices(1,1)],[obs.vertices(2,:)  obs.vertices(2,1)],[otherX  otherX(1)],[otherY  otherY(1)]);
           tf = ~isempty(xi);          
        end
    end
end