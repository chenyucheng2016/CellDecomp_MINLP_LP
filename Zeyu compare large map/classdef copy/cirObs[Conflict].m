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
        
        
        
        % determines if a line cross this obj
        function tf = ifLineCross2(obs,lineX,lineY,varargin)  %x = [0 10]; y = [4 0];
            robotx = 0; roboty = 0; targetx = 6; targety = 6;
            xs = [targetx robotx];
            ys = [targety  roboty];
            if isempty(xout)
                tf = false;
            else
                if reachTargetDist(xout,yout,robotx,roboty,targetx,targety)
                    tf = false;  % line NOT cross
                else
                    tf = true;
                end
            end
            if ~isempty(varargin)
                mapshow(lineX,lineY,'Marker','+')
                mapshow(xs,ys,'DisplayType','polygon','LineStyle','none');
                if ~isempty(xi)
                     mapshow(xi,yi,'DisplayType','point','Marker','o');
                end
            end
        end
        
        

        % flag ture, can see target
        function flag = cutLine(xout,yout,lineX1,lineX2,lineY1,lineY2)
            flag = true;
            for i = 1:length(xout)
                % if any intersection on the line
                if  pointOnLine( xout(i),yout(i),lineX1,lineX2,lineY1,lineY2)
                    flag = false;
                    return
                end
            end
        end


        
        % check if a point is on the line segment given by lineX lineY
        function tf = pointOnLine(pointX,pointY,lineX1,lineX2,lineY1,lineY2)
            tol = 0.0001; % tolerance
            distance = dist(pointX,pointY, lineX1,lineY1) + dist(pointX,pointY, lineX2,lineY2) - dist(lineX2,lineY2, lineX1,lineY1);
            tf = (distance<tol);
        end

        % convert from two points to slope and intersect of line
        function [k,b] = slopeIntercept(xs,ys)
            k = (ys(2)-ys(1))/(xs(2)-xs(1));
            b = ys(1)-k*xs(1);
        end
        
        % distance between two points
        function d = dist(x1,y1,x2,y2)
            d = sqrt((y2-y1)^2+(x2-x1)^2);
        end
        
                        % determines if a line cross this obj
        function tf = ifLineCross(obs,lineX,lineY,varargin)  %x = [0 10]; y = [4 0];
            % circle           
            cx = obs.loc(1); cy =  obs.loc(2); r = obs.rad;
            c = [cx   cy];
            pos = [c-r 2*r 2*r];
            k = (lineY(2)-lineY(1))/(lineX(2)-lineX(1));
            b = lineY(1)-k*lineX(1);
            %[k,b] = slopeIntercept(lineX,lineY);
            [xout,yout] = linecirc(k,b,cx,cy,r);
                        
            if isempty(xout)
                tf = false;
            else
                if cutLine(xout,yout,lineX(1),lineX(2),lineY(1),lineY(2))
                    tf = false;  % line NOT cross
                else
                    tf = true;
                    if ~isempty(varargin)
                        plot(xout,yout,'ro');
                    end
                end
            end
            if ~isempty(varargin)
                rectangle('Position',pos,'Curvature',[1 1],'EdgeColor','k')
            end
        end
        
        
        

    end
end