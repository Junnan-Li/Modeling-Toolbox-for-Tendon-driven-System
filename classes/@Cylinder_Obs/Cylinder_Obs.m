%% Class of Obstacles

% Descriptions: 
%       properties:
%           name:    name of the muscle
%           axis:   axis of height, z axis by default
%           radius:   radius of the cylinder
%           height:   length of the cylinder
%                       
%       functions: 
%           plot_obs:   plot the cylinder
% 


classdef Cylinder_Obs < Obstacles
    
    properties (Access = public)
        axis
        radius
        height
    end
    properties (SetAccess = private)

    end
    
    methods
        function obj = Cylinder_Obs(name)
            % Obstacle initialize
            obj = obj@Obstacles(name)
            obj.axis = [0,0,1];
            obj.radius = 0;
            obj.height = 0;
        end

        function plot_obs(obj, plot_par)

            if plot_par.inhand == 0
                w_T_obs = obj.w_T_Obs;
            else
                w_T_obs = obj.w_T_Obs_inhand;
            end
            [X,Y,Z] = cylinder_vertics_to_plot(w_T_obs, obj.radius,obj.height,10);
            s = surf(X,Y,Z,'FaceColor','g', 'EdgeColor','none');
            alpha(s,.6)
            hold on
            axis equal
        end
    end
end

