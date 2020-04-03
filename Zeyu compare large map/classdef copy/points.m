classdef points
    properties
        loc

    end
    
    methods(Access = public)
        % constructor
        function obj = points(loc,t)
            if nargin > 0  % allows calls of constructor with no arguments, e.g. a(1,7) = points(7), 1-6 has not arguments
                obj.loc = loc;

            else
                obj.loc = NaN;

            end
            
        end
        
        
        function plotPt(obj,varargin)
            if nargin < 2
                plot(obj.loc(1),obj.loc(2));
            else
                plot(obj.loc(1),obj.loc(2),varargin{:});
            end
        end
        
        % determine if two points can be connected without crossing
        % obstales
        function flag = ifConnect(obj,otherPt,obstacles)
            flag = true;
            for groupObs = obstacles  % e.g. groupObs = walls
                for item = groupObs{1}  % e.g. wall #1  
                    lineX = [obj.loc(1)  otherPt.loc(1)];
                    lineY = [obj.loc(2)  otherPt.loc(2)];
                    if ifLineCross(item,lineX,lineY) % ifLineCross in cirObs and recObs
                        flag = false;  
                        % if any obstacles block line of two Pts,
                        % not connected
                        return
                    end
                end
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