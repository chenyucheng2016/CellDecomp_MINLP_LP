classdef points
    properties
        loc
        type
    end
    
    methods(Access = public)
        % constructor
        function obj = points(loc,t)
            if nargin > 0  % allows calls of constructor with no arguments, e.g. a(1,7) = points(7), 1-6 has not arguments
                obj.loc = loc;
                obj.type = t;
            else
                obj.loc = NaN;
                obj.type = NaN;
            end
            
        end
        
        
        function plotPt(obj,varargin)
            if nargin < 2
                plot(obj.loc(1),obj.loc(2));
            else
                plot(obj.loc(1),obj.loc(2),varargin{:});
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