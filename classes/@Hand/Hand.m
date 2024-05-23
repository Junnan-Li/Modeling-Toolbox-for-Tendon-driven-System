%% Class of Hand:
% 



classdef Hand < handle & matlab.mixin.Copyable
    %FINGER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        name                % [char] name of the hand

        q                   % [nj] joint angle of all fingers

        nf                  % [1] number of fingers
        nj                  % [1] number of joints
        nja                 % [1] number of active joints    
        nl                  % [1] number of links
        nc                  % [1] number of contacts
        nt                  % [1] number of tendons
        nmus                % [1] number of muscles
        nvia                % [1] number of viapoints

        list_fingers        % [] list of all fingers
        list_joints         % [njx1] list of all joints
        list_links          % [nlx1] list of all links
        list_contacts       % [ncx1] list of all contacts
        list_tendons        % [ntx1] list of all tendons
        list_muscles        % [ntx1] list of all muscles
        list_viapoints      % [ntx1] list of all viapoints

        wrist               % [] a figner object connect all fingers
        
        w_p_base            % position of the base in world frame. Default: [0,0,0]'
        w_R_base            % rotation of the base in world frame. Default: non rotation

        topology
    end
    
    properties (SetAccess = private)

    end
    
    methods
        function obj = Hand(name)
            % 
            obj.name = name;
            % init lists    
            

            obj.q = [];
            obj.nf = 0;
            obj.list_fingers = [];
            obj.nj = 0;
            obj.list_joints = [];
            obj.nl = 0;
            obj.list_links = [];
            obj.nc = 0;
            obj.list_contacts = [];
            obj.nt = 0;
            obj.list_tendons = [];
            obj.nmus = 0;
            obj.list_muscles = [];
            obj.nvia = 0;
            obj.list_viapoints = [];
            

            obj.w_p_base = [0;0;0];
            obj.w_R_base = eye(3);

        end


        function add_finger(obj, finger)
            % add finger to hand  
            assert(isa(finger,'Finger'), '[add_finger] input is not a Finger object!');
            obj.list_fingers = [obj.list_fingers; finger];
            obj.q = [];
            obj.update_hand_list; 

        end

        function update_hand_list(obj)
            % update all list
            obj.nf = length(obj.list_fingers);
            obj.nj = 0;
            obj.nl = 0;
            obj.nc = 0;
            obj.nt = 0;
            obj.nmus = 0;
            obj.nvia = 0;
            for i = 1:obj.nf
                finger_i = obj.list_fingers(i);
                obj.nj = obj.nj + finger_i.nj;
                obj.nl = obj.nl + finger_i.nl;
                obj.nc = obj.nc + finger_i.nc;
                obj.nt = obj.nt + finger_i.nt;
                obj.nmus = obj.nmus + finger_i.nmus;
                obj.nvia = obj.nvia + finger_i.nvia;
            end
            obj.q = zeros(obj.nj,1);
            obj.update_hand(obj.q);
        end

        function update_hand(obj, q)
            % update hand configuration
            assert(length(q) == obj.nj, '[update_hand] dimension of q is incorrect!');
            obj.q = q;
            q_i_start = 1;
            for i = 1:obj.nf
                finger_i = obj.list_fingers(i);
                nj_i = finger_i.nj;
                q_i = q(q_i_start:q_i_start+nj_i-1);
                finger_i.update_finger(q_i);
                q_i_start = q_i_start + nj_i;
            end
        end
        

        function set_base(obj, w_p_base, w_R_base)
            % set the position and orientation of the base in the world
            % frame
            obj.w_p_base = w_p_base;
            obj.w_R_base = w_R_base;
%             obj.update_finger(obj.q_a);
        end

        function set_figner_base(obj, w_p_base, w_R_base, finger_index)
            % set the position and orientation of the base in the world
            % frame
            assert(finger_index <= obj.nf, '[set_figner_base] finger_index is too large!');
            
            finger_i = obj.list_fingers(finger_index);
            finger_i.set_base( w_p_base, w_R_base);
            finger_i.update_finger(finger_i.q);
        end


        %% visualization


        function plot_hand(obj,varargin)

%             if nargin == 1
%                 parameters = obj.plot_parameter_init;
%             elseif nargin == 2
%                 parameters =  varargin{1};
%             else
%                 error('[plot_finger] input dimension is incorrect! \n')
%             end

            for i = 1:obj.nf
                finger_i = obj.list_fingers(i);
                finger_i.plot_finger();
            end

        end




    end
end

