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
            plot(robot.loc(1), robot.loc(2),'bs','MarkerFaceColor','r','MarkerSize',8);
        end
        
        % draw two fan region which is the robot FOV
        function drawFOV(robot) 
            drawFan(robot,[robot.loc(1)  robot.loc(2)], robot.fov(2), robot.fov(3),robot.loc(3));
            drawFan(robot,[robot.loc(1)  robot.loc(2)], robot.fov(1), robot.fov(3),robot.loc(3));
        end
        
        % Helper function for drawFOV
        function drawFan(robot,startLoc,r,theta,direction)
            x0 = startLoc(1);
            y0 = startLoc(2);
            direction1 = direction - theta/2;
            direction2 = direction + theta/2;
            t = linspace(direction1,direction2);
            x = x0 + r*cos(t);
            y = y0 + r*sin(t);
            plot([x0,x,x0],[y0,y,y0],'k-')
            axis equal
        end
        
        % Determine if a target in the FOV  (seems to work)
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
        
        % Determine if any of targets is in the FOV --> can define
        % C-target
        function boolean = inCTarget(robot,targetSet)
            boolean = false;
            for i = targetSet
                if ptInFOV(robot,i)
                    boolean = true;
                end
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
        
    end
end