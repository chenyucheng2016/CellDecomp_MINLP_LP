classdef obstacles
    properties
        vertices
        type
    end
    
    methods
        % constructor
        function obj = obstacles(v,t)
            
            if nargin > 0  % allows calls of constructor with no arguments, e.g. a(1,7) = points(7), 1-6 has not arguments
                obj.vertices = v;
                obj.type = t;
            else
                obj.vertices = NaN;
                obj.type = NaN;
            end
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
    end
    
    
end