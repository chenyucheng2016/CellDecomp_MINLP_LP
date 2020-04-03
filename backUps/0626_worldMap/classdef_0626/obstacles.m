classdef obstacles
    properties
%         vertices
        type
    end
    
    methods
        % constructor
        function obj = obstacles(t)
            
            if nargin > 0  % allows calls of constructor with no arguments, e.g. a(1,7) = points(7), 1-6 has not arguments
%                 obj.vertices = v;
                obj.type = t;
            else
%                 obj.vertices = NaN;
                obj.type = NaN;
            end
        end
        
       
    end
    
    
end