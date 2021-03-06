classdef robot
    properties
        loc  % x, y , theta(in rad)
        fov  % includes radius range(minR, maxR), and angle of FOV
        size
    end
    
    methods
        % constructor
        function obj = robot(loc,fov,size)
            
            if nargin > 0  % allows calls of constructor with no arguments
                obj.loc = loc;
                obj.fov = fov;
                obj.size = size;
            else
                obj.loc = NaN;
                obj.fov = NaN;
                obj.size = NaN;
            end
        end
        
        
        function plotRobot(robot)
%             plot(robot.loc(1), robot.loc(2),'bs','MarkerFaceColor','r','MarkerSize',8);
            xs = [robot.loc(1)-robot.size(1)  robot.loc(1)-robot.size(1)  robot.loc(1)+robot.size(1)  robot.loc(1)+robot.size(1)];           
            ys = [robot.loc(2)-robot.size(2)  robot.loc(2)+robot.size(2)  robot.loc(2)+robot.size(2)  robot.loc(2)-robot.size(2)];          
            fill(xs,ys,'k');
        end
        
        % draw two fan region which is the robot FOV
        function drawFOV(robot) 

            drawFan(robot,[robot.loc(1)  robot.loc(2)], robot.fov(2), robot.fov(3),robot.loc(3),'r');
            hold on
            for i=1:10  % stupid way to avoid effect of alpha transparency
            drawFan(robot,[robot.loc(1)  robot.loc(2)], robot.fov(1), robot.fov(3),robot.loc(3),'w');
            end
        end
        
        % Helper function for drawFOV
        function drawFan(robot,startLoc,r,theta,direction,color)
            x0 = startLoc(1);
            y0 = startLoc(2);
            direction1 = direction - theta/2;
            direction2 = direction + theta/2;
            t = linspace(direction1,direction2);
            x = x0 + r*cos(t);
            y = y0 + r*sin(t);
            plot([x0,x,x0],[y0,y,y0],'k-')
            fill([x0,x,x0],[y0,y,y0],color)
            axis equal
        end
        
        % %% C-free and its helper funtions below====================
        
        % determine if a robot position is in C-free
        % Currently assume UGV does not rotate, horizontal (but the
        % camera rotates) will need function to convert from angle and
        % center to vertices if rotates UGV
        % obstacles can be struct, 3 groups of obstacles
        function boolean = inCFree(robot,obstacles)
            [robotXs,robotYs] = locSize2Vertices(robot);
            hold on
%              plot([robotXs   robotXs(1)],[robotYs   robotYs(1)],'g');
            % check if vertices out of bound
            for i = length(robotXs)
                if robotXs(i)<0 || robotYs(i)<0 || robotXs(i)>9.2 || robotYs(i)>9.2 
                    boolean = false;
                    return
                end
            end
            
            for groupObs = obstacles  % e.g. groupObs = walls
                for item = groupObs{1}  % e.g. wall #1
                    if ptInObs(item,robot.loc(1),robot.loc(2)) % if center is in obstacle
                        boolean = false;
                        return
                    end
                    if polyConflict(item,robotXs,robotYs) % if polygon intersect
                        boolean = false;
                        return
                    end
                    
                end
            end
            boolean = true;
        end
        
        
        % Currently assume UGV does not rotate, horizontal (but the
        % camera rotates) will need function to convert from angle and
        % center to vertices if rotates UGV
        function [robotXs,robotYs] = locSize2Vertices(robot)
            dx = robot.size(1)/2;  dy = robot.size(2)/2;
            cx = robot.loc(1); cy = robot.loc(2);
            robotXs = [cx-dx  cx+dx  cx+dx   cx-dx];
            robotYs = [cy+dy  cy+dy  cy-dy   cy-dy];
        end
        
        
        % rotate the robot and check if the robot is in C-free
        
        % %% C-free and its helper funtions end====================
        
        % %% C-target and its helper funtions below====================
        % Determine if a target in the FOV  (not rotating robot)
        function flag = ptInFOV(robot, pt)
            flag = false;
            % deals with angle
            phi = anglePtRobot(robot,pt);
            rbtRot = robot.loc(3); 
            difAngle = abs(wrapToPi(phi - rbtRot));
%             difAngle = wrapToPi(phi - rbtRot);
%             disp(radtodeg(difAngle))
            if difAngle < robot.fov(3)/2
                flag = true;               
            end
            
            % deals with radius/ distance  (TESTED)
            d = distPtRobot(robot,pt);
            if d > robot.fov(1) && d < robot.fov(2)               
            else
                flag = false;
            end
        end
        
        % Determine if a target in the FOV  (rotating robot)
        function flag = ptInFOV_rot(robot, pt,obstacles)
            flag = false;


            rotStep = pi/12;
            origOrient = robot.loc(3);
            for rotation = 0:rotStep:2*pi
                robot.loc(3) = origOrient + rotation;
                if ptInFOV(robot, pt) && ~ifBlocked(obstacles,robot,pt)                    
                    flag = true;
                    return
                end
            end
        end
        
        % Given set of obstacles(now for recObs only) and robot and target,
        % determine if line crosses any obstacles
        function flag = ifBlocked(obstacles,robot,target)
            flag = false;
            for groupObs = obstacles  % e.g. groupObs = walls
                for item = groupObs{1}  % e.g. wall #1  
                    lineX = [robot.loc(1)  target.loc(1)];
                    lineY = [robot.loc(2)  target.loc(2)];
                    if ifLineCross(item,lineX,lineY) % ifLineCross in cirObs and recObs
                        flag = true;  
                        % if any obstacles block robot and target, return true
                        return
                    end
                end
            end
        end
        
%          % determines if a line cross this obj
%         function tf = ifLineCross(obs,lineX,lineY,varargin)  %x = [0 10]; y = [4 0];
%             
%             xs = obs.vertices(1,:);
%             ys = obs.vertices(2,:);
%             
%             [xi, yi] = polyxpoly(lineX, lineY, xs, ys);
%             if isempty(xi)
%                 tf = false;
%             else
%                 tf = true;
%                
%             end
%            
%             if ~isempty(varargin)
%                  mapshow(lineX,lineY,'Marker','+')
%                 mapshow(xs,ys,'DisplayType','polygon','LineStyle','none');
%                 if ~isempty(xi)
%                      mapshow(xi,yi,'DisplayType','point','Marker','o');
%                 end
%             end
%  
%         end
%         
%         
%   
        % Determine if any of targets is in the FOV --> can define
        % C-target
        function boolean = inCTarget(robot,targetSet,obstacles)
            boolean = false;
            for i = targetSet
                if ptInFOV_rot(robot,i,obstacles)
                    boolean = true;
                    break
                end
%                 if ptInFOV(robot,i)
%                     boolean = true;
%                     break
%                 end
            end
        end 
        
        % angle of target wrt the robot, CCW+
        function phi = anglePtRobot(robot,pt)  
              phi = atan2(pt.loc(2) - robot.loc(2), pt.loc(1) - robot.loc(1));  
        end
        
        % distance of target wrt the robot loc
        function d = distPtRobot(robot,pt)  
              d = sqrt((pt.loc(1)-robot.loc(1))^2+(pt.loc(2)-robot.loc(2))^2);
        end
        % %% C-target and its helper funtions end====================
        
    end
end