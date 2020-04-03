classdef points
    properties
        myX
        myZ
        type
    end
    
    methods(Access = public)
        % constructor
        function obj = points(x,z,t)
            if nargin > 0  % allows calls of constructor with no arguments, e.g. a(1,7) = points(7), 1-6 has not arguments
                obj.myX = x;
                obj.myZ = z;
                obj.type = t;
            else
                obj.myX = NaN;
                obj.myZ = NaN;
                obj.type = NaN;
            end
            
        end
        
        
        function plotPt(obj,varargin)
            if nargin < 2
                plot(obj.myX,obj.myZ);
            else
                plot(obj.myX,obj.myZ,varargin{:});
            end
        end
    end
    
    
    
    %    methods
    %       function r = roundOff(obj)
    %          r = round([obj.Value],2);
    %       end
    %       function r = multiplyBy(obj,n)
    %          r = [obj.Value] * n;
    %       end
    %    end
end