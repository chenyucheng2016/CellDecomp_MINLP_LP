% DirectionalSensor.m
% Jake Gemerek
% 12/6/18
%
% Map contains details for a workspace, such as the workspace limits,
% target positions, and obstacle polygons.
%
classdef directional_sensor
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
        % sensor position
        x;
        % sensor rotation discretization
        theta;
        % sensor FOV opening angle in radians
        alpha;
        % discretization of curved region in sector FOV
        nu_vec;
        % maximum sensor range
        r_max;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        % -------------------------- CONSTRUCTOR ------------------------ %
        function obj = directional_sensor(x,theta,alpha,r_max,n)
            obj.x = x;
            obj.theta = theta;
            obj.alpha = alpha;
            obj.nu_vec = linspace(-obj.alpha/2,obj.alpha/2,n);
            obj.r_max = r_max;
        end
        % --------------------------- FUNCTIONS ------------------------- %
       
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end










