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
%                       to be implemented
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
%             obj.ma_value = [];
%             obj.stiffness = 0; %
%             obj.damping = 0; %
            obj.n_vp = 0; %
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
                list_constr_tmp{i} = obj.list_vp(i);
                list_constr_index_tmp(i) = obj.list_vp(i).Link.index;
            end
            for i = 1:obj.n_obs
                obj_i = obj.list_obs{i};
                if obj_i.Link.index == 0
%                     index_i = 1;
                    list_constr_tmp = {obj_i,list_constr_tmp{:}}';
                    list_constr_index_tmp = [0;list_constr_index_tmp(:)];
                else
                    index_i = min(find(list_constr_index_tmp == obj_i.Link.index));
                    list_constr_tmp = {list_constr_tmp{1:index_i-1},obj_i,list_constr_tmp{index_i:end}}';
                    list_constr_index_tmp = [list_constr_index_tmp(1:index_i-1); ...
                                    obj_i.Link.index; list_constr_index_tmp(index_i:end)];
                end
%                 list_constr_tmp{index_i+1:end} = list_constr_tmp{index_i:end-1};
%                 list_constr_tmp{index_i,1} = obj_i;
            end
            obj.list_constr = list_constr_tmp(~cellfun(@isempty, list_constr_tmp));
        end

        function switch_constr(obj, index_con1, index_con2)
            % manually 
            list_constr_tmp = obj.list_constr;
            list_constr_tmp(index_con1) = list_constr_tmp(index_con2);
            list_constr_tmp(index_con2) = obj.list_constr(index_con1);
            obj.list_constr = list_constr_tmp;
        end



        %% calculate length
        function l = cal_muscle_length(obj)
            % calculate the muscle length (L2 norm)
            l = 0;
            for i = 1:obj.n_vp-1
                p_vp_i = obj.list_vp(i).w_p_VP;
                p_vp_inext = obj.list_vp(i+1).w_p_VP;
                p_dis_i = norm(p_vp_inext-p_vp_i);
                l = l + p_dis_i;
            end
        end
          
        function l = cal_muscle_length_inhand(obj)
            % calculate the muscle length (L2 norm)
            l = 0;
            for i = 1:obj.n_vp-1
                p_vp_i = obj.list_vp(i).w_p_VP_inhand;
                p_vp_inext = obj.list_vp(i+1).w_p_VP_inhand;
                p_dis_i = norm(p_vp_inext-p_vp_i);
                l = l + p_dis_i;
            end
        end

        %% plot 
        function plot_muscles(obj,plot_par) 
            % plot muscles with associated via points and obstacles
            if obj.n_vp == 0
                fprintf('[plot_muscles]: muscle %s has no via point! /n', obj.name)
            else
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

            if obj.n_obs == 0
%                 fprintf('[plot_muscles]: muscle %s has no via point! /n', obj.name)
            else
                for i = 1:obj.n_obs
                    obs_i = obj.list_obs{i};
                    obs_i.plot_obs(plot_par);
                end
            end
        end
    end
end

