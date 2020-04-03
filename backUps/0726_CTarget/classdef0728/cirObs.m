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
        
        % determines if a line cross this obj (TESTED)
        function tf = ifLineCross(obs,lineX,lineY,varargin)  %x = [0 10]; y = [4 0];
            % circle
            cx = obs.loc(1); cy =  obs.loc(2); r = obs.rad;
            c = [cx   cy];
            pos = [c-r 2*r 2*r];
            
            [k,b] = slopeIntercept(obs,lineX,lineY);
            [xout,yout] = linecirc(k,b,cx,cy,r);
            
            if isempty(xout)
                tf = false;
            else
                tf = cutLine(obs,xout,yout,lineX,lineY);
                if tf == true
                    if ~isempty(varargin)
                        plot(xout,yout,'ro');
                    end
                end
            end
            if ~isempty(varargin)
                hold on
                rectangle('Position',pos,'Curvature',[1 1],'EdgeColor','k')
                plot(lineX,lineY);
            end
        end
        % %% ifLineCross helper functions below %%%%%%=================
        function flag = cutLine(obs,xout,yout,lineX,lineY)
            flag = false;
            xv = [lineX(1)  lineX(1)  lineX(2)  lineX(2)];
            yv = [lineY(1)  lineY(1)  lineY(2)  lineY(2)];
            for i = 1:length(xout)
                if  inpolygon(xout(i),yout(i),xv,yv)
                    flag = true;
                    return
                end
            end
        end

        
        
        % check if a point is on the line segment given by lineX lineY
        function tf = pointOnLine(obs,pointX,pointY,lineX,lineY)
            tol = 0.0001; % tolerance
            distance = dist1(obs,pointX,pointY, lineX(1),lineY(1)) + dist1(obs,pointX,pointY, lineX(2),lineY(2)) - dist1(obs,lineX(2),lineY(2), lineX(1),lineY(1));
            tf = (distance<tol);
        end
        % distance between two points
        function d = dist1(obs,x1,y1,x2,y2)
            d = sqrt((y2-y1)^2+(x2-x1)^2);
        end
        
        % convert from two points to slope and intersect of line
        function [k,b] = slopeIntercept(obs,xs,ys)
            k = (ys(2)-ys(1))/(xs(2)-xs(1));
            b = ys(1)-k*xs(1);
        end
        % %% ifLineCross helper functions end  %%%%%%==================
        
        % determine if a point is in this obstacle
        function tf = ptInObs(obs,ptX,ptY)
            cirX = obs.loc(1); cirY = obs.loc(2);
            tf = (dist1(obs,cirX,cirY,ptX,ptY)<= obs.rad);
        end
        
        % circle polyConflict:
        % current: no vertices are in cirObs. otherX and Y are vertices
        function tf = polyConflict(obs,otherX,otherY)         
            for i = 1:length(otherX)
                if ptInObs(obs,otherX(i),otherY(i))
                    tf = true;
                    return
                end
                tf = false;
            end
        end
    end
end