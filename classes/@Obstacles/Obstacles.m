%% Class of Obstacles

% Comment: 
%       properties:
%           name:    name of the muscle
%           links:   the link object that obstacle attached 
% 
%       functions: 
%           

classdef Obstacles < handle
    
    properties (Access = public)
        name                 % [char] name of tendon
        link_p              % [3x1] position in the link frame
        link_R              % [3x3] orientation in the link frame
    end
    properties (SetAccess = private)
        link
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
        
        function add_Obs_to_link(obj,link_obj)
            obj.link = link_obj;
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
        function w_T_Obs_inhand = get_w_T_Obs_inhand(obj)
            w_T_Obs_inhand = obj.w_T_Obs_inhand;
        end

    end
end

