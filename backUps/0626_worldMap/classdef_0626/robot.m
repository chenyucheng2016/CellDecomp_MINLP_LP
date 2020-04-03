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

        % Determine if target in the FOV
%         function flag = ptInFOV(obj,point)   % point is a points object
        function flag = isInFOV(phi, robotRotation,FOV)
            
            robotRotation = robotRotation + pi;
            difangle = abs(wrapToPi(phi - robotRotation));
            if difangle < FOV/4
                flag = true;
            else
                flag = false;
            end
            
        end
        
        function [phi] = angleRobotPt(robot,pt)      
            node_trans = wb_supervisor_node_get_field(object,'translation');
            position = wb_supervisor_field_get_sf_vec3f(robot_trans);
            pt_pos = [pt.];
            
            phi = atan2(position(1) - pt_pos(1),position(3) - pt_pos(3));
            phi = atan2(pt_pos(1) - position(1), pt_pos(3) - position(3));
            dV = pt_pos - position;
            phi = atan2(dV(1),dV(3));
%             [position;object_position;]

    end

    end
    
    
end