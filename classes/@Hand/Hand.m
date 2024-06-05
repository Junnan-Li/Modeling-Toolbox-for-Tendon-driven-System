%% Class of Hand:
% 
% properties
%       Base                % [Finger] a serial Finger object
%       list_finger         % [nf] the list of all attached finger
%       list_links          % [nl] the list of all attached finger
%       list_muscles        % [nmus] the list of all attached finger
%       list_viapoints      % [nvia] the list of all attached finger
%       list_tendons        % [nt] the list of all attached finger
% 
%       w_p_base            % position of the base in world frame. taken
%                           from Base properties 
%       w_R_base            % rotation of the base in world frame.  taken
%                           from Base properties 
% 
%       njb + njf = nj      

% 
% 


classdef Hand < handle & matlab.mixin.Copyable
    %FINGER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        name                % [char] name of the hand

        q                   % [nj] joint angle of all fingers
        nb                  % [1] number of base
        nf                  % [1] number of fingers (exclude base)
        nj                  % [1] number of joints (include base)
        njb                 % [1] number of joints of base
        njf                 % [1] number of joints of fingers
        nja                 % [1] number of active joints    
        nl                  % [1] number of links (include base)
%         nlb                 % [1] number of links of base
        nc                  % [1] number of contacts
        nt                  % [1] number of tendons
        nmus                % [1] number of muscles
        nvia                % [1] number of viapoints
        base                % [1] the base of hand (wrist)
        list_fingers        % [] list of all fingers
        list_joints         % [njx1] list of all joints
        list_links          % [nlx1] list of all links
        list_contacts       % [ncx1] list of all contacts
        list_tendons        % [ntx1] list of all tendons
        list_muscles        % [ntx1] list of all muscles
        list_viapoints      % [ntx1] list of all viapoints
        
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
            obj.nb = [];
            obj.base = [];
            obj.nf = 0;
            obj.list_fingers = [];
            obj.nj = 0;
            obj.njb = 0;
            obj.njf= 0;
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
            obj.list_fingers = [obj.list_fingers; finger.copy];
%             obj.q = [];
            obj.update_hand_list; 
        end

        function add_base(obj, finger)
            % add finger to hand  
            assert(isa(finger,'Finger'), '[add_finger] input is not a Finger object!');
            obj.base = finger.copy;
            obj.update_hand_list; 
            obj.update_hand(obj.q); 
        end

        function update_hand_list(obj)
            % update all list
            obj.nf = length(obj.list_fingers);
            obj.q = [];
            obj.nb = length(obj.base);
            obj.nj = 0;
            obj.njb = 0;
            obj.njf = 0;
            obj.nl = 0;
            obj.nc = 0;
            obj.nt = 0;
            obj.nmus = 0;
            obj.nvia = 0;

            % update base information
            for i = 1:obj.nb
                base_i  = obj.base(i);
                obj.nj = obj.nj + base_i.nj;
                obj.njb = obj.njb + base_i.nj;
                obj.nl = obj.nl + base_i.nl;
                obj.nc = obj.nc + base_i.nc;
                obj.nt = obj.nt + base_i.nt;
                obj.nmus = obj.nmus + base_i.nmus;
                obj.nvia = obj.nvia + base_i.nvia;
                obj.q = [obj.q;base_i.q];
                
            end
            for i = 1:obj.nf
                finger_i = obj.list_fingers(i);
                obj.nj = obj.nj + finger_i.nj;
                obj.njf = obj.njf + finger_i.nj;
                obj.nl = obj.nl + finger_i.nl;
                obj.nc = obj.nc + finger_i.nc;
                obj.nt = obj.nt + finger_i.nt;
                obj.nmus = obj.nmus + finger_i.nmus;
                obj.nvia = obj.nvia + finger_i.nvia;
                obj.q = [obj.q;finger_i.q];
            end
%             obj.update_hand(obj.q);
        end

        function update_hand(obj, q)
            % update hand configuration
            % and update the w_p_base & w_R_base for all bases and fingers
            assert(length(q) == obj.nj, '[update_hand] dimension of q is incorrect!');
            obj.q = q;
%             w_p_base_i = obj.w_p_base; % w to base of hand 
%             w_R_base_i = obj.w_R_base;
            W_T_b_prior = obj.get_W_T_B;
            if obj.njb ~= 0
                q_b = q(1:obj.njb); % joint of base
                q_i_start = 1;% index for bases
                for i = 1:obj.nb
                    base_i = obj.base(i);
                    W_T_bi = base_i.get_W_T_B;
                    W_T_bi_new = W_T_b_prior*W_T_bi;
                    base_i.set_w_T_base_inhand(W_T_bi_new);

                    nj_i = base_i.nj;
                    q_i = q_b(q_i_start:q_i_start+nj_i-1);
                    base_i.update_finger(q_i);
                    W_T_b_prior = base_i.get_w_T_ee_inhand;
                    q_i_start = q_i_start + nj_i;
                end
            end
            if obj.njf ~= 0
                q_f = q(obj.njb+1:end); % joint of fingers
                q_i_start = 1; % index for fingers
                for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);

                    W_T_bi = finger_i.get_W_T_B;
                    W_T_bi_new = W_T_b_prior*W_T_bi;
                    finger_i.set_w_T_base_inhand(W_T_bi_new);
                    nj_i = finger_i.nj;
                    q_i = q_f(q_i_start:q_i_start+nj_i-1);
                    finger_i.update_finger(q_i);
                    q_i_start = q_i_start + nj_i;
                end
            end
        end
        

        function set_base(obj, w_p_base, w_R_base)
            % set the position and orientation of the base in the world
            % frame
            obj.w_p_base = w_p_base;
            obj.w_R_base = w_R_base;
            obj.base.setbase(w_p_base,w_R_base);
%             obj.update_finger(obj.q_a);
        end

        function W_T_b = get_W_T_B(obj)
            % get the homogeneous transformation matrix from Base to W
            W_T_b = [obj.w_R_base,obj.w_p_base; 0 0 0 1];
        end

        function set_figner_base(obj, w_p_base, w_R_base, finger_index)
            % set the position and orientation of the base in the world
            % frame
            assert(finger_index <= obj.nf, '[set_figner_base] finger_index is too large!');
            
            finger_i = obj.list_fingers(finger_index);
            finger_i.set_base( w_p_base, w_R_base);
            finger_i.update_finger(finger_i.q);
        end

        %% rst model

        function rst_model_hand = update_rst_model(obj)
            % update the rst model of hand and construct to one robotictree
            if obj.nb == 0
                disp('Hand has no base element!!')
                return
            end

            base_1 = obj.base(1).update_rst_model;
            rst_model_hand = base_1.copy;
            merge_bodyname = rst_model_hand.BodyNames{end};
            if obj.nb > 2
                for i = 2:obj.nb
                    base_i = obj.base(i).update_rst_model;
                    rst_model_hand.addSubtree(merge_bodyname,base_i);
                    merge_bodyname = rst_model_hand.BodyNames{end};
                end
            end
            if obj.nf == 0
                disp('Hand has no finger element!!')
            else
                for i = 1:obj.nf
                    rst_finger_i = obj.list_fingers(i).update_rst_model;
                    rst_model_hand.addSubtree(merge_bodyname,rst_finger_i);
                end
            end

        end


        %% visualization

        function parstr = plot_parameter_init(obj)
            % init the parameter struct for plot_finger
            % check plot_parameter_init.m
            parstr = plot_parameter_init();
            parstr.inhand = 1;
        end


        function plot_hand(obj,varargin)
            % plot the hand with parameter.inhand = 1;
            if nargin == 1
                pars = obj.plot_parameter_init();
            elseif nargin == 2
                pars =  varargin{1};
            else
                error('[plot_finger] input dimension is incorrect! \n')
            end
            pars.inhand = 1;
            % plot all bases
            if obj.njb ~= 0
                for i = 1:obj.nb
                    base_i = obj.base(i);
                    base_i.plot_finger(pars);
                end
            end
            % plot all fingers
            if obj.njf ~= 0
                for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    finger_i.plot_finger(pars);
                end
            end

        end

        

    end
end

