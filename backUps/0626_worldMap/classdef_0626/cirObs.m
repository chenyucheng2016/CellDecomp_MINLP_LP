classdef cirObs < obstacles
    properties
        loc
        rad
    end
    
    methods
         % constructor
        function cirObj = cirObs(loc,rad,t)
            if nargin > 2
                super_arg{1} = t;
            else
                super_arg{1} = NaN;
            end
            cirObj@obstacles(super_arg{:});
            if nargin > 2
                cirObj.loc = loc;
                cirObj.rad = rad;
            else
                cirObj.loc = NaN;
                cirObj.rad = NaN;
            end
        end

        
        % draws a single obstacle object, color (e.g.'k') and hollow(true) is optional.
        function plotObs(obs,color, hollow)
            
            r = obs.rad;
            %// center
            c = [obs.loc(1)   obs.loc(2)];         
            pos = [c-r 2*r 2*r];
            
            if nargin == 1
                rectangle('Position',pos,'Curvature',[1 1],'FaceColor','k');
            elseif nargin == 2
                rectangle('Position',pos,'Curvature',[1 1],'FaceColor',color);
            elseif  nargin == 3
                if hollow == true
                    rectangle('Position',pos,'Curvature',[1 1],'EdgeColor',color);
                end
            end
            alpha(.3)
        end
    end
end