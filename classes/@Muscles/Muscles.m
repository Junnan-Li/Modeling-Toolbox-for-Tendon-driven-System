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
        n_vp                 % [1] number of the viapoints
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
%             obj.ma_value = [];
%             obj.stiffness = 0; %
%             obj.damping = 0; %
            obj.n_vp = 0; %
%             obj.dyn_par = struct();
%             obj.dyn_par.stiffness = 1e3;
%             obj.dyn_par.damping = 0;
        end

        function add_viapoints(obj, viapoint_obj)
            % update the number of the viapoint
            obj.list_vp = [obj.list_vp; viapoint_obj];
            obj.update_viapoints;

        end

        function update_viapoints(obj)
            % update the number of the viapoint
            obj.n_vp = length(obj.list_vp);

        end
        
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
        

    end
end

