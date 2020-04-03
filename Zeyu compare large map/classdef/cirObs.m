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
            % sanity check, vertical line
            if k == inf || k == -inf
                if (min(lineY(:)) < cy) && (cy < max(lineY(:)))  % circle in between line start/end, start/end in circle case checked elsewhere
                    if dist2Line(obs,[cx cy],lineX,lineY)<=r
                        tf = true;
                        return
                    end
                end

            end
            [xout,yout] = linecirc(k,b,cx,cy,r);
            
            if isempty(xout)
                tf = false;
            end
            if ~any(~isnan(xout)) % all are xout
                tf = false;    
                return
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
        function dist = dist2Line(obs,xloc,lineX, lineY)
        x = xloc; %some point
        a = [lineX(1)  lineY(1)]; %segment points a,b
        b = [lineX(2)  lineY(2)];
        
        d_ab = norm(a-b);
        d_ax = norm(a-x);
        d_bx = norm(b-x);
        
        if dot(a-b,x-b)*dot(b-a,x-a)>=0
            A = [a,1;b,1;x,1];
            dist = abs(det(A))/d_ab;
        else
            dist = min(d_ax, d_bx);
        end
        end
        
        function flag = cutLine(obs,xout,yout,lineX,lineY)
            flag = false;
            xv = [lineX(1)  lineX(1)  lineX(2)  lineX(2)];
            yv = [lineY(1)  lineY(1)  lineY(2)  lineY(2)];
            for i = 1:length(xout)
                if isnan(xout(i))
                    continue
                end
%                 disp(xout(i));
%                 tf = inpolygon(xout(i),yout(i),xv,yv);
%                 error(lastwarn);
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
            tf = false;
            % no need to check if center in rect, other funtion will do it
            for i = 1:length(otherX)
                if ptInObs(obs,otherX(i),otherY(i))
                    tf = true;
                    return
                end
            end
            % now check if no edges cross the circle
            newXs = [otherX(:)'  otherX(1)];  newYs = [otherY(:)'  otherY(1)];  % close contour
            for i = 1:4
                lineX = [newXs(i)  newXs(i+1)]; lineY = [newYs(i)  newYs(i+1)];
                if ifLineCross(obs,lineX,lineY)
                    tf = true;
                    return
                end
            end
    
        end
        
        % detect if a circle intersects with another
        function tf = cirXcir(robotCir,item)
            x1 = robotCir.loc(1); y1 = robotCir.loc(2); r1 = robotCir.rad;
            x2 = item.loc(1); y2 = item.loc(2); r2 = item.rad;
            [xout,yout] = circcirc(x1,y1,r1,x2,y2,r2);
            tf = true; % circle conflict
            nan = [NaN NaN];
            if ~any(~isnan(xout))  % all are nan
                if dist1(robotCir,x1,y1,x2,y2)>= max(r1,r2)
                    tf = false;
                end
            end
        end
    end
end