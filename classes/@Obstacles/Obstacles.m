%% Class of Obstacles

% Descriptions: 
%       properties:
%           name:    name of the muscle
%           link_p:   position according to the link frame
%           link_R:   orientation according to the link frame
%           link:   the attached Link object (not used)
%           link_T_obs: 
%           base_T_obs:   transformation matrix according to the base frame,
%                         updated by Link.update_link_obstacles
%           w_T_Obs:    transformation matrix according to the world frame,
%                       updated by Finger.update_obstacles
%                       
%       functions: 
%           update_base_T_obs
%           update_w_T_Obs
%           plot_obs:   plot the obstacle origin and Z axis (override by subclass)
% 

classdef Obstacles < handle
    
    properties (Access = public)
        name                 % [char] name of tendon
        link_p              % [3x1] position in the link frame
        link_R              % [3x3] orientation in the link frame
    end
    properties (SetAccess = private)
        Link
        index_inhand        % index of the related link in q of hand
        link_T_obs
        base_T_obs          
        w_T_Obs          
        w_T_Obs_inhand
    end
    
    methods
        function obj = Obstacles(name)
            % Obstacle initialize
            obj.name = name;
            obj.link_p = nan(3,1);
            obj.link_R = nan(3,3);
            obj.link_T_obs = nan(4,4);
        end

        function set_index_inhand(obj,index_inhand)
            % 
            obj.index_inhand = index_inhand;
        end

        function add_Obs_to_link(obj,link_obj)
            obj.Link = link_obj;
        end

        function update_Obs_p_R(obj,link_p,link_R)
            % Obstacle link_T_obs
            obj.link_p = reshape(link_p,3,1);
            obj.link_R = link_R;
            obj.link_T_obs = [link_R,link_p; 0 0 0 1];
        end

        function update_base_T_obs(obj,base_T_link)
            % update the obj.base_T_obs with given base_T_link
            % called by Link.update_link_obstacles
            obj.base_T_obs = base_T_link * obj.link_T_obs;
        end
        function base_T_obs = get_base_T_obs(obj)
            % update the obj.base_T_obs with given base_T_link
            % called by Link.update_link_obstacles
            base_T_obs = obj.base_T_obs;
        end


        function update_w_T_Obs(obj, w_T_Obs)
            % update the obj.w_T_Obs with given base_T_link
            % called by Finger.update_obstacles
            obj.w_T_Obs = w_T_Obs;
        end

        function update_w_T_Obs_inhand(obj,w_T_Obs_inhand_new)
            obj.w_T_Obs_inhand = w_T_Obs_inhand_new;
        end

        function w_T_Obs = get_w_T_Obs(obj)
            w_T_Obs = obj.w_T_Obs;
        end
        function w_T_Obs_inhand = get_w_T_Obs_inhand(obj)
            w_T_Obs_inhand = obj.w_T_Obs_inhand;
        end

        function plot_obs(obj, plot_par)

            if plot_par.inhand == 0
                [p, R] = T2pR(obj.w_T_Obs);
            else
                [p, R] = T2pR(obj.w_T_Obs_inhand);
            end
            plot3(p(1),p(2),p(3),'*','Color',plot_par.markercolor,...
                                    'MarkerSize',plot_par.markersize);
            hold on

            if plot_par.axis_show
                W_p =  plot_par.axis_len*R(1:3,3); % z axis of obstacle frame
                h = quiver3(p(1),p(2),p(3),...
                    W_p(1),W_p(2),W_p(3),...
                    'Color','b','LineWidth',plot_par.linewidth);
                set(h,'AutoScale','on', 'AutoScaleFactor',1);
                hold on
                xlabel('x')
                ylabel('y')
                zlabel('z')
                axis equal
            end
        end

    end
end

