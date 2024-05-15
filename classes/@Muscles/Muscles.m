%% Class of Muscles:
% Muscle object with pathway informations enabling obstacle-set algorithms


% Comment: 
%       properties:
%           list_vp: a list of the via points, by default, the first one is origin,]
%                    the last one is insertion. 
% 
%               OriginPoint
%               Viapoints
%               InsertionPoint
% 
%       functions: 
%           set_via_point
%           delete_via_point
%           update_viapoints()
%           cal_muscle_length(q)



classdef Muscles < handle
    
    properties (Access = public)
        name                 % [char] name of tendon
        list_vp              % [] list of the via points
        n_vp                 % [1] number of the viapoints
        stiffness            % [1] stiffness
        damping              % [1] damping
        

    end
    properties (SetAccess = private)
%         nj_finger            % [1] number of joints from finger.nj 
%         index                % [1] index of the tendon of a finger
        ma_value             % [nj_fingerx1] moment arms in m, + for flexion 
        ma_limits            % [4xnj_fingerx2] minimal and maximal abs value of tendon
    end
    
    methods
        function obj = Muscles(name)
            % Muscle initialize
            obj.name = name;
%             obj.nj_finger = 0;
            obj.list_vp = [];
            obj.ma_value = [];
            obj.stiffness = 0; %
            obj.damping = 0; %
            obj.n_vp = 0; %


        end

        function update_viapoints(obj)
            % update the number of the viapoint
            obj.n_vp = length(obj.list_vp);

        end
        
        function l = cal_muscle_length(obj)
            % calculate the muscle length (L2 norm)
            l = 0;
            for i = 1:obj.n_vp-1
                p_vp_i = obj.list_vp(i).base_p;
                p_vp_inext = obj.list_vp(i+1).base_p;
                p_dis_i = norm(p_vp_inext-p_vp_i);
                l = l + p_dis_i;
            end
        end
          
        

    end
end

