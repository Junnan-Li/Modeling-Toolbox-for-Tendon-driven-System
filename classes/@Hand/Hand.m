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
        nobs                % [1] number of obstacles
        base                % [1] the base of hand (wrist)
        list_fingers        % [] list of all fingers
        list_joints         % [njx1] list of all joints
        list_links          % [nlx1] list of all links
        list_contacts       % [ncx1] list of all contacts
        list_tendons        % [ntx1] list of all tendons
        list_muscles        % [ntx1] list of all muscles
        list_viapoints      % [ntx1] list of all viapoints
        list_obstacles     % [ntx1] list of all obstacles
        
        limits_q            % [njx2] min,max values of q
        limits_qd           % [njx2] min,max values of qd
        limits_qdd          % [njx2] min,max values of qdd
        limit_j_on          % [1] activate the joint limits

        index_q_b           % [njb,2] start and end index of base joint in q
        index_q_f           % [njf,2] start and end index of base joint in q

        topology
    end
    
    properties (SetAccess = private)

        w_p_base            % position of the base in world frame. Default: [0,0,0]', always 
                            % equal to the w_p_base of the first base
        w_R_base            % rotation of the base in world frame. Default: non rotation
        par_dyn_h           % Dynamic parameters from all bases and fingers
        limit_joint_on      % [1] activate the joint limits
        state               % state for simulation: q, qd, l_mus, alpha
                            %   [obj.nj] q; [obj.nj] qd; [obj.nmus] l_mus; [obj.nmus] alpha;
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
            obj.nobs = 0;
            obj.list_obstacles = [];

            obj.w_p_base = [0;0;0];
            obj.w_R_base = eye(3);


            obj.limit_joint_on = 0;
            obj.limits_q = [];
            obj.limits_qd = [];
            obj.limits_qdd = [];

            obj.index_q_b = [];
            obj.index_q_f = [];
            
            obj.par_dyn_h = struct();
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
            obj.base = [obj.base;finger.copy];
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
            obj.nobs = 0;
            obj.limits_q = [];
            obj.limits_qd = [];
            obj.limits_qdd = [];
            
            obj.list_obstacles = {};
            obj.index_q_b = [];
            obj.index_q_f = [];
            
            i_q = 1;
            % update base information
            if obj.nb ~= 0
                obj.w_p_base = obj.base(1).w_p_base;
                obj.w_R_base = obj.base(1).w_R_base;
                obj.index_q_b = zeros(obj.nb,2);
                for i = 1:obj.nb

                    base_i  = obj.base(i);

                    obj.nj = obj.nj + base_i.nj;
                    obj.njb = obj.njb + base_i.nj;
                    obj.nl = obj.nl + base_i.nl;
                    obj.nc = obj.nc + base_i.nc;
                    obj.nt = obj.nt + base_i.nt;
                    obj.nmus = obj.nmus + base_i.nmus;
                    obj.nvia = obj.nvia + base_i.nvia;
                    obj.nobs = obj.nobs + base_i.nobs;
                    obj.q = [obj.q;base_i.q];
                    obj.limits_q = [obj.limits_q;base_i.limits_q];
                    obj.limits_qd = [obj.limits_qd;base_i.limits_qd];
                    obj.limits_qdd = [obj.limits_qdd;base_i.limits_qdd];
                    
                    % sammurize the Finger list to Hand list
                    obj.list_obstacles = {obj.list_obstacles{:},base_i.list_obstacles{:}}';

                    obj.index_q_b(i,:) = [i_q,i_q+ base_i.nj-1];
                    i_q = i_q + base_i.nj;
                end
            end
            if obj.nf ~= 0
                obj.index_q_f = zeros(obj.nf,2);
                for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    obj.nj = obj.nj + finger_i.nj;
                    obj.njf = obj.njf + finger_i.nj;
                    obj.nl = obj.nl + finger_i.nl;
                    obj.nc = obj.nc + finger_i.nc;
                    obj.nt = obj.nt + finger_i.nt;
                    obj.nmus = obj.nmus + finger_i.nmus;
                    obj.nvia = obj.nvia + finger_i.nvia;
                    obj.nobs = obj.nobs + finger_i.nobs;
                    obj.q = [obj.q;finger_i.q];
                    obj.limits_q = [obj.limits_q;finger_i.limits_q];
                    obj.limits_qd = [obj.limits_qd;finger_i.limits_qd];
                    obj.limits_qdd = [obj.limits_qdd;finger_i.limits_qdd];

                    obj.list_obstacles = {obj.list_obstacles{:},finger_i.list_obstacles{:}}';

                    obj.index_q_f(i,:) = [i_q,i_q+ finger_i.nj-1];
                    i_q = i_q + finger_i.nj;
                end
            end
            obj.update_list_viapoints;
            %             obj.update_hand(obj.q);
        end

        function update_hand(obj, q)
            % update hand configuration
            % and update the w_p_base_inhand & w_R_base_inhand for all 
            % bases and fingers
            assert(length(q) == obj.nj, '[update_hand] dimension of q is incorrect!');
            obj.q = q;
            W_T_b_prior = eye(4);
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
        
        %% joints
        function set_joint_limits_on(obj)
            % update joint information
            obj.limit_joint_on = 1;
            % update base information
            if obj.nb ~= 0
                for i = 1:obj.nb
                    obj.base(i).set_joint_limits_on();
                end
            end
            if obj.nf ~= 0
                for i = 1:obj.nf
                    obj.list_fingers(i).set_joint_limits_on();
                end
            end
        end
        function set_joint_limits_off(obj)
            % update joint information
            obj.limit_joint_on = 1;
            % update base information
            if obj.nb ~= 0
                for i = 1:obj.nb
                    obj.base(i).set_joint_limits_off;
                end
            end
            if obj.nf ~= 0
                for i = 1:obj.nf
                    obj.list_fingers(i).set_joint_limits_off;
                end
            end
        end

        function update_joint_limits(obj)
            % update joint information
            if obj.nb ~= 0
                for i = 1:obj.nb
                    base_i  = obj.base(i);
                    obj.limits_q = [obj.limits_q;base_i.limits_q];
                    obj.limits_qd = [obj.limits_qd;base_i.limits_qd];
                    obj.limits_qdd = [obj.limits_qdd;base_i.limits_qdd];
                end
            end
            if obj.nf ~= 0
                for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    obj.limits_q = [obj.limits_q;finger_i.limits_q];
                    obj.limits_qd = [obj.limits_qd;finger_i.limits_qd];
                    obj.limits_qdd = [obj.limits_qdd;finger_i.limits_qdd];
                end
            end
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
            if obj.nb > 1
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

        %% kinematics
        function set_base(obj, w_p_base, w_R_base)
            % set the position and orientation of the base in the world
            % frame
            obj.w_p_base = w_p_base;
            obj.w_R_base = w_R_base;
            obj.base(1).set_base(w_p_base,w_R_base);
%             obj.base(1).set_w_T_base_inhand(obj.base(1));
        end

        function W_T_b = get_W_T_B(obj)
            % get the homogeneous transformation matrix from Base to W
            W_T_b = [obj.w_R_base,obj.w_p_base; 0 0 0 1];
        end

        function set_figner_base(obj, w_p_base, w_R_base, finger_index)
            % set the position and orientation of the base in the world
            % need to use hand.update
            assert(finger_index <= obj.nf, '[set_figner_base] finger_index is too large!');
            
            finger_i = obj.list_fingers(finger_index);
            finger_i.set_base( w_p_base, w_R_base);
            finger_i.update_finger(finger_i.q);
        end

         function w_T_ee_all = get_w_T_ee_all(obj)
            % get the T of the all figners
            % [4*nf, 4]
            
            if obj.nf == 0
                w_T_ee_all = 0;
                disp('Hand has no finger element!!')
            else
                w_T_ee_all = zeros(4*obj.nf,4);
                for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    w_T_ee_all(4*i-3:4*i,:) = finger_i.get_w_T_ee_inhand;
                end
            end
         end

         function w_x_ee_all = get_w_x_ee_all(obj)
            % get the T of the all figners
            % [4*nf, 4]
            w_x_ee_all = zeros(6,obj.nf);
            w_T_ee_all = obj.get_w_T_ee_all;
            for i = 1:obj.nf
                w_T_ee_i = w_T_ee_all(4*i-3:4*i,:);
                [p,R] = T2pR(w_T_ee_i);
                w_x_ee_all(:,i) = [p; R2euler_XYZ(R)];
            end
         end

         function w_T_all = get_w_T_all(obj)
            % get the T of the all figners
            % [4*nf, 4]
%             W_T_b = obj.get_W_T_B();
            w_T_all = zeros(4,4,obj.nl+2*(obj.nb+obj.nf)); 
            % each Finger has base, link1, ... linkn, ee
            index = 1;
            if obj.nb ~= 0
                for i = 1:obj.nb
                    base_i = obj.base(i);
                    n_frame_i = base_i.nl+2;
                    w_T_all(:,:,index:index+n_frame_i-1) = base_i.get_T_all_links_inhand;
                    index = index+n_frame_i;
                end
            end
            if obj.nf ~= 0
                for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    n_frame_i = finger_i.nl+2;
                    w_T_all(:,:,index:index+n_frame_i-1) = finger_i.get_T_all_links_inhand;
                    index = index+n_frame_i;
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

        function plot_hand_com(obj,varargin) 
            % plot the center of mass of each link in world frame
            if nargin == 1
                par = obj.plot_parameter_init();
            elseif nargin == 2
                par =  varargin{1};
            else
                error('[Hand:plot_hand_com] input dimension is incorrect! \n')
            end

            if obj.njb ~= 0
                for i = 1:obj.nb
                    base_i = obj.base(i);
                    base_i.plot_com(par);
                end
            end
            if obj.njf ~= 0
                for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    finger_i.plot_com(par);
                end
            end
        end

        function plot_hand_viapoints(obj,varargin) 
            % plot the center of mass of each link in world frame
            if nargin == 1
                par = obj.plot_parameter_init();
            elseif nargin == 2
                par =  varargin{1};
            else
                error('[Hand:plot_hand_via] input dimension is incorrect! \n')
            end

            if obj.njb ~= 0
                for i = 1:obj.nb
                    base_i = obj.base(i);
                    base_i.plot_viapoints(par);
                end
            end
            if obj.njf ~= 0
                for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    finger_i.plot_viapoints(par);
                end
            end
        end

        function plot_hand_muscles(obj,varargin) 
            % plot the com of each link in world frame
            
            if nargin == 1
                plot_par = obj.plot_parameter_init;
            elseif nargin == 2
                plot_par =  varargin{1};
            else
                error('[Hand:plot_hand_muscles] input dimension is incorrect! \n')
            end

            for i = 1:obj.nmus
%                 w_p_viapoints_all = obj.get_p_muscle_viapoint(i);
%                 plot3(w_p_viapoints_all(1,:)',w_p_viapoints_all(2,:)',w_p_viapoints_all(3,:)','-',...
%                     'Color',par.muscle_linecolor,'MarkerSize',par.muscle_markersize,'LineWidth',par.muscle_linewidth);
%                 hold on
                muscle_i = obj.list_muscles(i);
                muscle_i.plot_muscles(plot_par);
            end
        end

        function plot_hand_obstacles(obj,varargin) 
            % plot the com of each link in world frame
            
            if nargin == 1
                par = obj.plot_parameter_init;
            elseif nargin == 2
                par =  varargin{1};
            else
                error('[Hand:plot_hand_muscles] input dimension is incorrect! \n')
            end

            if obj.nobs ~= 0
                for i = 1:obj.nobs
                    obs_i = obj.list_obstacles{i};
                    obs_i.plot_obs(par);
                end
            end
        end

        %% Dynamics
        
        function update_hand_par_dyn(obj)
            % update the finger dynamic parameters 
            if obj.njb ~= 0
                for i = 1:obj.nb
                    base_i = obj.base(i);
                    obj.par_dyn_h.mass_all{i,1} = base_i.par_dyn_f.mass_all;
                    obj.par_dyn_h.com_all{i,1} = base_i.par_dyn_f.com_all;
                    obj.par_dyn_h.inertia_all{i,1} = base_i.par_dyn_f.inertia_all;
                end
                obj.par_dyn_h.g = obj.base(1).par_dyn_f.g;
            end
            if obj.njf ~= 0
               for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    obj.par_dyn_h.mass_all{i,2} = finger_i.par_dyn_f.mass_all;
                    obj.par_dyn_h.com_all{i,2} = finger_i.par_dyn_f.com_all;
                    obj.par_dyn_h.inertia_all{i,2} = finger_i.par_dyn_f.inertia_all;
                end
            end
        end

        function set_g_w(obj,g)
            % g is in world frame as input 
            g_reshape = reshape(g,3,1);
            obj.par_dyn_h.g = g_reshape;
            if obj.njb ~= 0
                for i = 1:obj.nb
                    base_i = obj.base(i);
                    base_i.set_g_w(g);
                end
            end
            if obj.njf ~= 0
               for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    finger_i.set_g_w(g);
                end
            end
        end

        %% viapoints
        function update_list_viapoints(obj)
            % update the Link Class property: Links.nc 
            
            obj.list_viapoints = []; % init list_contacts
            num_viapoints = 0;
            if obj.nb ~= 0
                for i = 1:obj.nb
                    base_i = obj.base(i);
                    base_i.update_list_viapoints;
                    obj.list_viapoints = [obj.list_viapoints;base_i.list_viapoints];
                    num_viapoints = num_viapoints + base_i.nvia;
                end
            end
            if obj.nf ~= 0
               for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    finger_i.update_list_viapoints;
                    obj.list_viapoints = [obj.list_viapoints;finger_i.list_viapoints];
                    num_viapoints = num_viapoints + finger_i.nvia;
                end
            end
            obj.nvia = num_viapoints;
        end
        
        function w_p_viapoints_all = get_p_all_viapoints_inhand(obj)
            % plot 3d viapoints
            w_p_viapoints_all = NaN(3,obj.nvia);
            vp_index = 1;
            if obj.nb ~= 0
                for i = 1:obj.nb
                    base_i = obj.base(i);
                    w_p_viapoints_all(3,vp_index:vp_index+base_i.nvia-1) = ...
                        base_i.get_p_all_viapoints_inhand;
                    vp_index = vp_index + base_i.nvia;
                end
            end
            if obj.nf ~= 0
               for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    w_p_viapoints_all(3,vp_index:vp_index+base_i.nvia-1) = ...
                        finger_i.get_p_all_viapoints_inhand;
                    vp_index = vp_index + finger_i.nvia;
                end
            end
        end

        function delete_all_viapoint(obj)
            % delete all viapoints
            if obj.nb ~= 0
                for i = 1:obj.nb
                    base_i = obj.base(i);
                    base_i.delete_all_viapoint;
                end
            end
            if obj.nf ~= 0
               for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    finger_i.delete_all_viapoint;
                end
            end
            obj.update_list_viapoints;
        end
        %% Muscle  
        function Muscle_obj = add_Muscle(obj, name)
            % add muscle to the finger
            Muscle_obj = Muscles(name);
            obj.list_muscles = [obj.list_muscles;Muscle_obj];
            obj.nmus = length(obj.list_muscles);
        end

        function delete_Muscle(obj, index)
            % delete muscle to the finger
            obj.list_muscles(index) = [];
            obj.nmus = length(obj.list_muscles);
        end

        function w_p_viapoints_all = get_p_muscle_viapoint(obj, muscle_index)
            assert(isempty(find(muscle_index - obj.nmus > 0)),'[Hand:get_p_muscle_viapoints] muscle index exceeds the number of muscles!')
            % plot 3d muscle via points
            muscle_i = obj.list_muscles(muscle_index);
            w_p_viapoints_all = zeros(3,muscle_i.n_vp);
            if muscle_i.n_vp == 0
                fprintf('[Hand:get_p_muscle_viapoints] muscle %d has no viapoints! \n',muscle_index)
            end
            for j = 1:muscle_i.n_vp
                w_p_VP_inhand_i = muscle_i.list_vp(j).get_w_p_VP_inhand;
                w_p_viapoints_all(:,j) = w_p_VP_inhand_i;
            end
        end


        function l_muscle = get_muscle_length(obj, muscle_index)
            assert(isempty(find(muscle_index - obj.nmus > 0)),'[Hand:get_p_muscle_viapoints] muscle index exceeds the number of muscles!')
            % 
            l_muscle = zeros(length(muscle_index),1);
            for i = 1:length(muscle_index)
                muscle_i = obj.list_muscles(muscle_index(i));
                if muscle_i.n_vp == 0
                    fprintf('[Hand:get_p_muscle_viapoints] muscle %d has no viapoints! \n',muscle_index(i))
                end
                l_muscle(i) = muscle_i.cal_muscle_length_inhand;
            end
        end

        function l_muscle_all = get_muscle_length_all(obj)
            % 
            l_muscle_all = zeros(obj.nmus,1);
            for i = 1:obj.nmus
                l_muscle_all(i) = obj.get_muscle_length(i);
            end
        end

        function MA = get_Muscle_Momentarm_1st_c(obj,  varargin)
            % calculate the Momentarm matrix of all muscles
            % MA is a [nj,length(muscle_index)] matrix
            % calculate the MA value using first order centered 
            % numeric differentiation method

            if nargin == 1
                muscle_index = (1:obj.nmus)';
                step = 1e-3;
            elseif nargin == 2
                muscle_index = varargin{1};
                step = 1e-3;
            elseif nargin == 3
                muscle_index = varargin{1};
                step = varargin{2};
            end
            MA = zeros(obj.nj,length(muscle_index));
            q_ori = obj.q;
            for i = 1: obj.nj
%                 for j = 1: length(muscle_index)
                    q_f = q_ori;
                    q_f(i) = q_f(i) + step;
                    obj.update_hand(q_f);
                    l_f = obj.get_muscle_length(muscle_index); % forward 
                    q_b = q_ori;
                    q_b(i) = q_b(i) - step;
                    obj.update_hand(q_b);
                    l_b = obj.get_muscle_length(muscle_index); % forward 
                    MA(i,:) = (l_f-l_b)./(2*step); 
%                 end
            end
            obj.update_hand(q_ori);
        end

        function tau_mus = get_muscle_torque(obj, f_mus)
            % temporary function for calculate torque_muscle
            assert(length(f_mus) == obj.nmus, '[Hand.get_muscle_torque] input dimension is incorrect!')
            MA = obj.get_Muscle_Momentarm_1st_c;
            tau_mus = MA * f_mus;
        end

        %% obstacles
         function update_list_obstacles(obj)
            % update the Link Class property: Links.nc 
            obj.list_obstacles = {}; % init list_contacts
            num_obstacles = 0;
            % add viapoint from base
            if obj.nb ~= 0
                for i = 1:obj.nb
                    base_i = obj.base(i);
                    base_i.update_list_obstacles();
                    num_obstacles = num_obstacles + base_i.nobs;
                    if base_i.nobs~=0
                        obj.list_obstacles = {obj.list_obstacles{:},base_i.list_obstacles{:}}';
                    end
                end
            end
            if obj.nf ~= 0
               for i = 1:obj.nf
                    finger_i = obj.list_fingers(i);
                    finger_i.update_list_obstacles();
                    num_obstacles = num_obstacles + finger_i.nobs;
                    if finger_i.nobs~=0
                        obj.list_obstacles = {obj.list_obstacles{:},finger_i.list_obstacles{:}}';
                    end
                end
            end
            obj.nobs = num_obstacles;
            
        end       


        %% state
        function state = init_state(obj)
            % 
            q_init = obj.q;
            qd_init = zeros(obj.nj,1);
            l_mus_init = obj.get_muscle_length_all;
            alpha_init = zeros(obj.nmus,1);
            state = [q_init;qd_init;l_mus_init;alpha_init];
            obj.state = state;
        end

        function set_state(obj,state)
            % 
            obj.state = state;
            obj.update_hand(state(1:obj.nj));
        end

        function state = get_state(obj)
            % 
            state = obj.state;
        end


    end
end

