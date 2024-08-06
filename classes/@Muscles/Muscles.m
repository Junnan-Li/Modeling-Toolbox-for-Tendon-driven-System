%% Class of Muscles:
% Base Muscle object with pathway informations enabling obstacle-set algorithms


% Comment: 
%       properties:
%           name:    name of the muscle
%           list_vp: 
%                       current version: the first one is the origin, 
%                                       the last one is insertion point,
%                                       others are viapoints
%           list_obstacle: 
%                           list of all obstacles related to the obstacle
%           list_constr:
%                           list of constraints including viapoints and
%                           obstacles in muscle path sequence
%           constr_vp:      cell array of muscle path constrains only
%                           containing viapoints
%           constr_obs:     cell array of muscle path constrains containing
%                           containing obstacles and adjacent vps
%           
% 
%       functions: 
%           set_via_point
%           delete_via_point
%           update_viapoints()
%           cal_muscle_length(q)
%           cal_muscle_length_inhand
% 

classdef Muscles < handle
    
    properties (Access = public)
        name                 % [char] name of tendon
        list_vp              % [] list of the via points
        list_obs             % list of the obstacles
        list_constr          % list of constraints includes vp and obs
        constr_vp
        constr_obs
        n_vp                 % [1] number of the viapoints
        n_obs
    end
    properties (SetAccess = private)
%         nj_finger            % [1] number of joints from finger.nj 
%         index                % [1] index of the tendon of a finger
%         ma_value             % [nj_fingerx1] moment arms in m, + for flexion 
%         ma_limits            % [4xnj_fingerx2] minimal and maximal abs value of tendon
    end
    
    methods
        function obj = Muscles(name)
            % Muscle initialize
            obj.name = name;
%             obj.nj_finger = 0;
            obj.list_vp = [];
            obj.list_obs = {};
            obj.constr_vp = {};
            obj.constr_obs = {};
%             obj.ma_value = [];
%             obj.stiffness = 0; %
%             obj.damping = 0; %
            obj.n_vp = 0; %
            obj.n_obs = 0; %
%             obj.dyn_par = struct();
%             obj.dyn_par.stiffness = 1e3;
%             obj.dyn_par.damping = 0;
        end

        %% viapoints
        function add_viapoints(obj, viapoint_obj)
            % update the number of the viapoint
            obj.list_vp = [obj.list_vp; viapoint_obj];
            obj.update_viapoints;
        end

        function switch_viapoints(obj, index_vp1, index_vp2)
            % switch two viapoints
            list_vp_tmp = obj.list_vp;
            list_vp_tmp(index_vp1) = list_vp_tmp(index_vp2);
            list_vp_tmp(index_vp2) = obj.list_vp(index_vp1);
            obj.list_vp = list_vp_tmp;
            obj.update_viapoints;
        end

        function update_viapoints(obj)
            % update the number of the viapoint
            obj.n_vp = length(obj.list_vp);
        end

        %% obstacles
        function add_Muscle_Obstacles(obj, obstacles_obj)
            % update the number of the viapoint
            obj.list_obs = {obj.list_obs{:}, obstacles_obj}';
            obj.update_obstacles;
        end
            
        function update_obstacles(obj)
            % update the number of the viapoint
            obj.n_obs = length(obj.list_obs);
%             muscle.init_list_constr;
        end
        

        %% constraints

        function init_list_constr(obj)
            % init constraint list 
            %%% called by obj.update_obstacles

            list_constr_tmp = cell(obj.n_obs+obj.n_vp,1);
            list_constr_index_tmp = nan(obj.n_vp,1);
            for i = 1:obj.n_vp
                list_constr_tmp{i,1} = obj.list_vp(i);
                list_constr_index_tmp(i) = obj.list_vp(i).Link.index;
            end
            for i = 1:obj.n_obs
                obj_i = obj.list_obs{i};
                if obj_i.Link.index == 0
                    list_constr_tmp = {obj_i,list_constr_tmp{:}}';
                    list_constr_index_tmp = [0;list_constr_index_tmp(:)];
                else
                    index_i = min(find(list_constr_index_tmp >= obj_i.Link.index));
                    list_constr_tmp = {list_constr_tmp{1:index_i-1},obj_i,list_constr_tmp{index_i:end}}';
                    list_constr_index_tmp = [list_constr_index_tmp(1:index_i-1); ...
                                    obj_i.Link.index; list_constr_index_tmp(index_i:end)];
                end
%                 list_constr_tmp{index_i+1:end} = list_constr_tmp{index_i:end-1};
%                 list_constr_tmp{index_i,1} = obj_i;
            end
            obj.list_constr = list_constr_tmp(~cellfun(@isempty, list_constr_tmp));
            obj.update_constr_array;
        end

        function switch_constr(obj, index_con1, index_con2)
            % manually 
            list_constr_tmp = obj.list_constr;
            list_constr_tmp(index_con1) = list_constr_tmp(index_con2);
            list_constr_tmp(index_con2) = obj.list_constr(index_con1);
            obj.list_constr = list_constr_tmp;
        end

        function update_constr_array(obj)
            % manually
            obj.constr_vp = {};
            obj.constr_obs = {};
            con_obs_index = 1;
            con_vp_index = 1;
            constr_comb_vp_i = {};
            for i = 1:length(obj.list_constr)
                IsObstacle_i = isa( obj.list_constr{i}, 'Obstacles' );
                if IsObstacle_i
                    obj.constr_obs{con_obs_index,1} = {obj.list_constr{i-1}, ...
                        obj.list_constr{i}, ...
                        obj.list_constr{i+1}}';
                    con_obs_index = con_obs_index + 1;
                    if length(constr_comb_vp_i) > 1
                        obj.constr_vp{con_vp_index,1} = constr_comb_vp_i;
                        con_vp_index = con_vp_index + 1;
                    end
                    constr_comb_vp_i = {};

                else
                    constr_comb_vp_i = {constr_comb_vp_i{:}, obj.list_constr{i}}';
                end
            end
            % check the last constr_vp_i has more than 2 vps
            if length(constr_comb_vp_i) > 1
                obj.constr_vp{con_vp_index,1} = constr_comb_vp_i;
                con_vp_index = con_vp_index + 1;
            end
        end

        %% muscle path
        
        function [l_total, wrap_status,w_PS_p] = cal_Muscle_length_ObstacleSet_Cyl_Garner(obj, varargin)
            % calculate the muscle length with Cylinder obstacle set
            % using cal_obstacle_vp_cyl_Garner.m 
            if nargin <= 1
                in_hand = 0;
            else
                in_hand = varargin{1};
            end
            l_total = 0;
            if ~isempty(obj.constr_vp)
                l_vp = nan(length(obj.constr_vp),1);
                for i = 1:length(obj.constr_vp)
                    %  a constraint list of straight line 
                    constr_vp_i = obj.constr_vp{i};
                    n_vp_i = length(constr_vp_i); % number of via points
                    w_p_vp = nan(3,n_vp_i);
                    for vp_index = 1:n_vp_i
                        vp_i = constr_vp_i{vp_index};
                        if in_hand == 0
                            w_p_vp(:,vp_index) = vp_i.get_w_p_VP;
                        else
                            w_p_vp(:,vp_index) = vp_i.get_w_p_VP_inhand;
                        end 
                    end
                    l_vp(i) = cal_muscle_vp_length(w_p_vp);
                    l_total = l_total + l_vp(i);
                end
            end
            if ~isempty(obj.constr_obs)
                l_obs = nan(length(obj.constr_obs),1);
                w_PS_p = cell(length(obj.constr_obs),1);
                wrap_status = nan(length(obj.constr_obs),1);
                for i = 1:length(obj.constr_obs)
                    constr_obs_i = obj.constr_obs{i};
                    if length(constr_obs_i)==3
                        if in_hand == 0
                            w_P_p = constr_obs_i{1}.get_w_p_VP;
                            w_T_obs = constr_obs_i{2}.get_w_T_Obs;
                            w_S_p = constr_obs_i{3}.get_w_p_VP;
                        else
                            w_P_p = constr_obs_i{1}.get_w_p_VP_inhand;
                            w_T_obs = constr_obs_i{2}.get_w_T_Obs_inhand;
                            w_S_p = constr_obs_i{3}.get_w_p_VP_inhand;
                        end
                        radius = constr_obs_i{2}.radius;
                        wrap_direction = sign(constr_obs_i{2}.axis(3));
                        [l_QT_i,wrap_status(i),w_Q_p,w_T_p] = cal_obstacle_vp_cyl_Garner(w_T_obs, w_P_p, w_S_p, radius, wrap_direction);
                        if wrap_status(i)
                            w_PS_p{i,1} = [w_Q_p,w_T_p];
                        end
                    else
                        disp('[Muscle.cal_Muscle_length_ObstacleSet_Cyl_Garner]: constr_obs has more than 3 elements! ')
                    end
                    l_obs(i) = l_QT_i;
                    l_total = l_total + l_QT_i;
                end
            end
        end


%         function l = cal_muscle_length(obj)
%             % calculate the muscle length (L2 norm)
%             l = 0;
%             for i = 1:obj.n_vp-1
%                 p_vp_i = obj.list_vp(i).w_p_VP;
%                 p_vp_inext = obj.list_vp(i+1).w_p_VP;
%                 p_dis_i = norm(p_vp_inext-p_vp_i);
%                 l = l + p_dis_i;
%             end
%         end
          
%         function l = cal_muscle_length_inhand(obj)
%             % calculate the muscle length (L2 norm)
%             l = 0;
%             for i = 1:obj.n_vp-1
%                 p_vp_i = obj.list_vp(i).w_p_VP_inhand;
%                 p_vp_inext = obj.list_vp(i+1).w_p_VP_inhand;
%                 p_dis_i = norm(p_vp_inext-p_vp_i);
%                 l = l + p_dis_i;
%             end
%         end

        %% plot 
        function plot_muscles(obj,plot_par) 
            % plot muscles with associated via points and obstacles

            % plot the obstacles 
            if obj.n_obs ~= 0
                for i = 1:obj.n_obs
                    obs_i = obj.list_obs{i};
                    obs_i.plot_obs(plot_par);
                end
            end
            
            if obj.n_vp == 0
                fprintf('[plot_muscles]: muscle %s has no via point! /n', obj.name)
            else
                if plot_par.muscle_plot_obstacleset && obj.n_obs ~= 0
                    % plot muscle wrapping over the obstacle set
                    [~, wrap_status,w_PS_p] = obj.cal_Muscle_length_ObstacleSet_Cyl_Garner(plot_par.inhand);
                    for i = 1:length(obj.constr_obs)
                        constr_obs_i = obj.constr_obs{i};
                        if wrap_status(i) == 1
                            % the wrapping occurs
                            if plot_par.inhand == 0
                                w_p_vp_1 = constr_obs_i{1}.get_w_p_VP;
                                w_p_vp_2 = constr_obs_i{3}.get_w_p_VP;
                            else
                                w_p_vp_1 = constr_obs_i{1}.get_w_p_VP_inhand;
                                w_p_vp_2 = constr_obs_i{3}.get_w_p_VP_inhand;
                            end
                            w_p_P_Q = [w_p_vp_1,w_PS_p{i}(:,1)];
                            w_p_T_S = [w_PS_p{i}(:,2),w_p_vp_2];

                            % plot muscle, via points and obstacle via
                            % points
                            plot3(w_p_P_Q(1,:)',w_p_P_Q(2,:),w_p_P_Q(3,:),...
                            '-', 'LineWidth',plot_par.muscle_linewidth,...
                            'Color',plot_par.muscle_linecolor)
                            plot3(w_p_T_S(1,:)',w_p_T_S(2,:),w_p_T_S(3,:),...
                            '-', 'LineWidth',plot_par.muscle_linewidth,...
                            'Color',plot_par.muscle_linecolor,'MarkerSize',plot_par.viapoint_markersize,...
                            'MarkerFaceColor',plot_par.viapoint_markercolor)
                            % plot vp 
                            plot3(w_p_vp_1(1),w_p_vp_1(2),w_p_vp_1(3),...
                            '.', 'Color',plot_par.viapoint_markercolor,'MarkerSize',plot_par.viapoint_markersize);
                            plot3(w_p_vp_2(1),w_p_vp_2(2),w_p_vp_2(3),...
                            '.', 'Color',plot_par.viapoint_markercolor,'MarkerSize',plot_par.viapoint_markersize); 
                            % plot obstacle vp by different color
                            plot3(w_PS_p{i}(1,:)',w_PS_p{i}(2,:),w_PS_p{i}(3,:),...
                            '.', 'Color',plot_par.obstacle_vp_color,'MarkerSize',plot_par.viapoint_markersize)   

                        else
                            % plot a straight line from the first to last
                            % vp in the constr_obs
                            if plot_par.inhand == 0
                                w_p_vp = [constr_obs_i{1}.get_w_p_VP,constr_obs_i{3}.get_w_p_VP];
                            else
                                w_p_vp = [constr_obs_i{1}.get_w_p_VP_inhand,constr_obs_i{3}.get_w_p_VP_inhand];
                            end
                            plot3(w_p_vp(1,:)',w_p_vp(2,:),w_p_vp(3,:),...
                            '.-', 'LineWidth',plot_par.muscle_linewidth,...
                            'Color',plot_par.muscle_linecolor,'MarkerSize',plot_par.viapoint_markersize)
                        end
                    end
                    for i = 1:length(obj.constr_vp)
                        constr_vp_i = obj.constr_vp{i};
                        w_p_vp_i = nan(3,length(constr_vp_i));
                        for j = 1:length(constr_vp_i)
                            if plot_par.inhand == 0
                                w_p_vp_j = constr_vp_i{j}.get_w_p_VP;
                            else
                                w_p_vp_j = constr_vp_i{j}.get_w_p_VP_inhand;
                            end
                            w_p_vp_i(:,j) = w_p_vp_j;
                        end
                        % plot the vp straight line
                        plot3(w_p_vp_i(1,:)',w_p_vp_i(2,:)',w_p_vp_i(3,:)',...
                            '-', 'LineWidth',plot_par.muscle_linewidth,...
                            'Color',plot_par.muscle_linecolor)
                        plot3(w_p_vp_i(1,:)',w_p_vp_i(2,:)',w_p_vp_i(3,:)',...
                            '.','Color',plot_par.viapoint_markercolor, ...
                            'MarkerSize',plot_par.viapoint_markersize)
                    end

                else
                    % just plot the straight line across vps
                    w_p_vp_all = nan(3,obj.n_vp);
                    for i = 1:obj.n_vp
                        vp_i = obj.list_vp(i);
                        vp_i.plot_viapoints(plot_par);
                        if plot_par.inhand == 0
                            w_p_VP_i = vp_i.w_p_VP;
                        else
                            w_p_VP_i = vp_i.w_p_VP_inhand;
                        end
                        w_p_vp_all(:,i)= w_p_VP_i;
                        plot3(w_p_vp_all(1,:)',w_p_vp_all(2,:)',w_p_vp_all(3,:)',...
                            '-', 'LineWidth',plot_par.muscle_linewidth,...
                            'Color',plot_par.muscle_linecolor)
                    end
                end
            end
        end
    end
end

