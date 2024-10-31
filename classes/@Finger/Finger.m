%% Class of Finger:
% Finger object represents a serial robot connected by revolute joint.
% For validation, the Finger object has a avatar model generated in Robotic
% System Toolbox from Matlab. 
%       After defininh the Finger object, use update_rst_model.m to generate 
%       rst model. 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% How to create a Finger object:
%       1. finger = Finger(finger_name, finger_type, link_len_vector)
%           finger_name: [char]           
%           finger_type: 
%               'RRRR': general finger with four joints(Ab/duction, MCP, PIP, DIP)
%               'RRRp': planar finger with three joints(MCP, PIP, DIP; ABD joint is fixed)
%               'RRRs': spatial finger with three joints(Ab/duction, MCP, PIP; DIP joint is fixed)
%           link_len_vector:
%               vector of the length of the links that match the number
%               the finger_type (mdh: a)
%       2. finger = Finger(finger_name, 'mdh',mdh_struct);
%           finger_name: [char] 
%           mdh_struct: structure contains mdh parameters; use
%               mdh_matrix_to_struct(mdh_matrix,1) to derive
%       3. create_finger_random(finger_name, finger_dof)
%           create a random finger with name and DoF. Normally used for
%           initializing a finger with random parameters and then modifying
%           the mdh parameters
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Coordinates:   
%           Coord. World: (w) world coordinate. symmetrical for all classes
%           Coord. base:  original coordinate for the **Class Finger**. 
%                         Tranformation information regarding w: w_p_base & w_R_base
%           Coord. links(joints) and endeffector: mdh related   
%%%%%%%%%%%%%%%%%%%%%%%%%
%       Links(Finger.base & Finger.list_links):
%           Base: a link object define the base of a finger with parameters
%               saved in Finger.base
%           Links: a link object for each link connected by joints saved in
%               Finger.list_links
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%       Kinematic parameters (saved as structure with dimension [nj+1,4])
%       NOTE: theta parameters are set to 0 initially and considered as q
%           Finger.mdh: alpha, a, theta (q), d 
%           Finger.mdha_all: same as Finger.mdh (current version)
%           Finger.mdh_ori: alpha, a, theta (without q), d 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Dynamic parameters (Finger.par_dyn_f)
%           .g: [3x1] gravity vector ([0;0;-9.81] by default)
%           .mall_all: [nl+1,1] mass value of base and links
%           .inertia_all: [6,nl+1]
%           .com_all: [3,nl+1]
%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Base/Link functions
%           w_T_base:  transformation matrix of Finger base to Finger world frame
%           w_T_base_inhand:  transformation matrix of Finger base to Hand
%               world frame
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Functions:
%           obj.update_finger(q_a):
%               with given q. this function sets q to obj.mdh.theta and 
%               udpates: 
%                       Link.base_p, Link.base_R, and associated 
%                       Contact, Viapoint, and Obstacles on that Link
%                       Link.w_T_Link, w_T_Link_inhand 
%               and calls: 
%                   update_viapoints.m, 
%                   update_M_coupling.m, 
%                   update_obstacles.m 
% 
% 
%           obj.update_finger_par_dyn:
%               updates obj.par_dyn_f from Links.par_dyn
% 
%           obj.update_joints_info:
%               updates obj.limits_q* with Joints.q*_limits
%               
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
%   Kinematics:
%       position
%           get_W_T_B: get base transformation matrix
%           get_p_all_links: get cartesian position of all link frames
%           get_T_all_links: get transformation matrix of all link frames
%           
%       Jacobian
%           Jacobian_geom_b_end
%           Jacobian_geom_w_end
%       
%       FK
%           update_finger(varargin)
% 
%       IK: 
%           IK_par: class of ik parameters for solvers
%           invkin_numeric: Newton-Raphson algorithm
%           invkin_numeric_LM: Levenberg-marquart algorithm
% 
%   functions are tested in test_ik.m   
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%   Dynamics:
%       general:
%           invdyn_ne_mdh: inverse dynamic using Newton-Euler with mdh as
%                          kinematic parameters, f_ext is [6,1]
%           invdyn_ne_mdh_sym: generate symbolic version of ID 
%           invdyn_ne_xq_mdh_all_fext: floating base with system state [x;q]
%           invdyn_ne_xq_mdh_wt_fext_sub: floating base with individual
%                                       M, C, G, Mqdd, F_Tau
%           invdyn_ne_xq_mdh_wt_fext_sub_sym_par: use parfor to speed up 
% 
%       Finger:
%           invdyn_ne_w_end: call invdyn_ne_mdh.m, for fixed base 
%           invdyn_ne_w_end_sym: call invdyn_ne_mdh_sym.m, has options for
%                               different sets of symbolic variables, for
%                               fixed base
%           invdyn_ne_xq_fb_all_fext: call invdyn_ne_xq_mdh_all_fext
%           invdyn_ne_xq_fb_all_fext_sym: call
%                                invdyn_ne_xq_mdh_all_fext_sym.m.
%           invdyn_ne_xq_fb_wt_fext_sub_sym
%           invdyn_ne_xq_fb_wt_fext_sub_sym_par
%           
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Simulink functions
%       kinematics:
%           sym_T_all_links_from_q: generate Transformation matrix of all
%               frames with q input
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%   tendon related: 
%       add_tendon();
%       update_M_coupling()
%       set_tendon_par_MA_poly3(obj, tendon_index, joint_index, par)
%
%   muscle related:
%       add_Muscle: add muscle to the Finger 
%       get_p_muscle_viapoints: get position of viapoints of muscle 
%           % todo: call Viapoint.w_p_VP to get position
%       cal_all_Muscle_Length: calculate all muscle length (call muscle.cal_muscle_length)
%       cal_selected_Muscle_Length
%       cal_all_Muscle_momentarm_1st_c(step): calculate the moment arm using
%           first order centered numeric differentiation method with given
%           step
%       cal_selected_Muscle_momentarm_1st_c
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%   viapoint related
%       Finger. add_ViaPoint(obj, name, link_index, link_p_obj): add viapoint to a
%           link of given index (0: base)  
%       update_list_viapoints(obj): update Finger.list_viapoints and Finger.nv

%       update_viapoints: update the w_p_viapoint for all viapoints, using 
%           Finger.get_p_viapoint_inhand and viapoint_i.update_w_p_VP.
%           called in Finger.update_finger

%       get_p_all_viapoints: calculate the position of viapoints in wowrld frame
%           and base frame

%       get_p_viapoint
%       get_p_all_viapoints_inhand
%       get_p_viapoint_inhand
%       delete_all_viapoint: delete all viapoints and empty list_viapoints
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Obstacle related:
%       add_Obstacle(obj, name, link_index, link_p_obj,link_R_obj): add
%           obstacle to the link of given index and related position and orientation
%       
%       update_list_obstacles: update Finger.list_obstacles and Finger.nobs
% 
%       update_obstacles: update the transformation matrix of each
%           obstacles (obstacles.update_w_T_Obs and .update_w_T_Obs_inhand)
%       get_p_obstacle: get position of the obstacle frame origin
%       get_T_obstacle: get transformation of the obstacle frame origin
% TODO: 
%       1. passive joints
%       2. function check_finger_properties 
%       3. dh parameter as another option to define kinematics
% 


classdef Finger < handle & matlab.mixin.Copyable    
    properties (Access = public)
        name                % [char] name of the finger
        type                % [char] finger type: 'RRRR', 'RRRp','RRRs'
        nj                  % [1] number of joints
        nja                 % [1] number of active joints    
        nl                  % [1] number of links
        nc                  % [1] number of contacts
        nt                  % [1] number of tendons
        nmus                % [1] number of muscles
        nvia                % [1] number of viapoints
        nobs                % [1] number of obstacles

        list_joints         % [njx1] list of all joints
        list_links          % [nlx1] list of all links
        list_contacts       % [ncx1] list of all contacts
        list_tendons        % [ntx1] list of all tendons
        list_muscles        % [ntx1] list of all muscles
        list_viapoints      % [ntx1] list of all viapoints
        list_obstacles      % [ntx1] list of all viapoints
        joint_act           % [njx1] logical vector: 1: active joint; 0: fixed joint 
        limits_q            % [njx2] min,max values of q
        limits_qd           % [njx2] min,max values of qd
        limits_qdd          % [njx2] min,max values of qdd
        
        limits_ft           % [ntx2] min,max values of tendon forces
        limits_t_ma         % [njx2] min,max values of moment arm for each joint
        
        par_kin             % all kinematic parameters
        opt_pkin            % kinematic parameters for optimization
        base                % base link                
        
    end
    
    properties (SetAccess = private)
        index_finger_inhand % the index of this Finger in a Hand Object 
        w_p_base            % position of the base in world frame. Default: [0,0,0]'
        w_R_base            % rotation of the base in world frame. Default: non rotation
        w_T_base            % transformation matrix of base frame in Finger world frame
        w_T_base_inhand     % transformation matrix of base frame in Hand world frame
        limit_joint_on      % [1] activate the joint limits
        mdh                 % mdh parameters: alpha, a theta, d (with q)
        mdh_all
        mdh_ori             % mdh parameters: alpha, a theta, d (without q)
        rst_model           % robotic system toolbox model based on mdh. Default: colume Dataformat
        q_a                 % [njax1] unit rad
        q                   % [njx1] unit rad
        par_dyn_f           % [nj+1x1]mass_all, [3xnj+1]com_all, [6xNJ+1] inertial regarding com, [3x1] g 
        M_coupling          % [njxnt] coupling matrix of the finger

%         w_T_links_all
    end
    
    methods
        function obj = Finger(name, varargin)
            % input syntax with priority:
            %   1. mdh struct: [alpha, a theta, d] without q
            %   2. finger_type: 
            %            
            obj.name = name;
            obj.mdh_ori = [];
            
            % Finger constructor input
            ip = inputParser;
            %    validScalarPosNum = @(x) isnumeric(x) && isscalar(x) && (x > 0);
            ip.addParameter('mdh',[]);
            ip.addParameter('type',[]);
            ip.addParameter('l_links',[]);
            parse(ip,varargin{:});
            p = ip.Results;
            
            if ~isempty(p.mdh)
                obj.type = 'customized';
                obj.mdh_ori = p.mdh;
                obj.nj = length(obj.mdh_ori.a)-1; %
                obj.nl = obj.nj;
                link_len_input = obj.mdh_ori.d;
            elseif ~isempty(p.type)
                type_input_split = split(p.type,'_');
                if strcmp(type_input_split{1},'R')
                    obj.type = p.type;
                    obj.mdh_ori = p.mdh; % empty 
                    link_len_input = p.l_links;
                    obj.nj = count(type_input_split{2},'R');
                    obj.nl = obj.nj;
                elseif strcmp(type_input_split{1},'H') % TODO
                    obj.type = p.type;
                    obj.mdh_ori = p.mdh;
                    link_len_input = p.l_links;
                    obj.nj = 4;
                    obj.nl = 4;
                end
            end
            
            % for R_... type
            % generate links
            list_links = [];
            for i = 1:obj.nl
                link_name_i = strcat(obj.name,'_Link',num2str(i));
                Link_new = Links(link_name_i,link_len_input(i), i);
                list_links = [list_links;Link_new];
            end
            obj.list_links = list_links;

            obj.base = Links(strcat(obj.name,'_base'),0,0);
            obj.base.update([0,0,0]',eye(3));
            % generate joints 
            list_joints = [];
            for i = 1:obj.nj
                Joint_name_i = strcat(obj.name,'_Joint',num2str(i));
                Joint_new = Joints(Joint_name_i,i);
                list_joints = [list_joints;Joint_new];
            end
            obj.list_joints = list_joints;
            obj.nja = obj.nj;
            obj.joint_act = ones(obj.nj,1); % TODO:passive joints
            
            % update mdh_ori
            if isempty(obj.mdh_ori)
                obj.mdh_ori = obj.get_finger_mdh_ori;       
            end
            
            obj.mdh = obj.mdh_ori;
            obj.mdh_all = obj.mdh_ori;
            
            % init list of contact and tendon
            % default contacts in the middle of each links
            obj.nc = 0;
            obj.list_contacts = [];
            
            obj.nt = 0;
            obj.list_tendons = [];
            obj.limits_t_ma = [];
            
            obj.nmus = 0;
            obj.list_muscles = [];   

            obj.nvia = 0;
            obj.list_viapoints = [];

            obj.nobs = 0;
            obj.list_obstacles = {};

            obj.index_finger_inhand = [];
            
            obj.limit_joint_on = 0;
            obj.limits_q = zeros(obj.nj,2);
            obj.limits_qd = zeros(obj.nj,2);
            obj.limits_qdd = zeros(obj.nj,2);
            % init finger base position and orientation
            obj.w_p_base = [0;0;0];
            obj.w_R_base = eye(3);
            obj.w_T_base = pR2T(obj.w_p_base,obj.w_R_base);
            obj.w_T_base_inhand = obj.w_T_base;
            obj.q_a = zeros(obj.nja,1);
            obj.q = zeros(obj.nj,1);
            % update mdh parameters
            obj.par_dyn_f = struct();
            obj.par_dyn_f.g = [0;0;-9.81];
            obj.par_dyn_f.mass_all = zeros(obj.nj+1,1);
            obj.par_dyn_f.inertia_all = zeros(6,obj.nj+1);
            obj.par_dyn_f.com_all = zeros(3,obj.nj+1);
            % set mdh parameters
%             obj.set_mdh_parameters(mdh_matrix)
            
            % update dynamic parameters
            obj.update_finger_par_dyn;
            obj.update_joints_info;
            % update parameters w.r.t. joint angles
            obj.update_finger(obj.q_a);
            
%             obj.update_rst_model;
        end
        
        function set_index_inhand(obj,index_inhand)
            % 
            obj.index_finger_inhand = index_inhand;
        end
        %% class property check
        function check_finger_properties(obj)
            % check if all the properties of the Finger class are in the
            % correct form
            % TODO 
        end
        
        %% kinematic related
        function set_mdh_parameters(obj, mdh_matrix)
            % set the mdh parameters with the given values 
            % sequence: alpha, a theta, d
            assert(size(mdh_matrix,1)== obj.nj+1, 'dimension of mdh_matrix is incorrect!')
            assert(size(mdh_matrix,2)== 4, 'dimension of mdh_matrix is incorrect!')
            mdh_ori_new = struct();
            mdh_ori_new.alpha = mdh_matrix(:,1);
            mdh_ori_new.a = mdh_matrix(:,2);
            mdh_ori_new.theta = mdh_matrix(:,3);
            mdh_ori_new.d = mdh_matrix(:,4);
%             mdh_ori_new.theta(:) = 0;
            obj.mdh_ori = mdh_ori_new;
            for i = obj.nl
                obj.list_links(i).Length = mdh_ori_new.a(i+1);
            end
            mdh_new = mdh_ori_new;
            mdh_new.theta = [obj.q;0];
            obj.mdh = mdh_new;
            obj.mdh_all = mdh_new;
%             obj.update_rst_model;
        end
        
        function mdh_ori = get_finger_mdh_ori(obj)
            % get mdh parameter from the given joint angle
            % update mhl parameters and par_kin, opt_pkin, ...

            % update mdh parameters 
            
            mdh_ori = struct;
            mdh_ori.alpha = zeros(obj.nj+1,1);
            mdh_ori.a = zeros(obj.nj+1,1);% [0;0;obj.list_links(1).Length;obj.list_links(2).Length;obj.list_links(3).Length];
            mdh_ori.theta = zeros(obj.nj+1,1); % 
            mdh_ori.d = zeros(obj.nj+1,1);
            for i = 1:obj.nj
                mdh_ori.a(i+1) = obj.list_links(i).Length;
            end
        end

        function update_finger(obj, varargin)
            % update mhl parameters and par_kin, opt_pkin, ...
            
            if nargin == 1
                q_new = obj.q;
            elseif nargin == 2
                q_new = varargin{1};
                assert(length(q_new)== obj.nja, '[update_finger]: dinput dimension is incorrect!')
                q_new = reshape(q_new,[obj.nj,1]);
            end
            
            if obj.limit_joint_on
                % saturate q with joint limits
                q_sat = vec_saturation(q_new,obj.limits_q);
                obj.q = q_sat; 
                obj.q_a = q_sat;
            else
                obj.q = q_new; % update joint angle
                obj.q_a = q_new;
            end

%             obj.q(obj.joint_act) = q_a; % update joint angle
            % update mdh parameters
%             [obj.mdh,obj.mdh_all, obj.mdh_ori] = get_finger_mdh(obj, obj.q_a);
            
            obj.mdh_all.theta = obj.mdh_ori.theta +  [obj.q;0];
            obj.mdh = obj.mdh_all;
            % update all link properties
            for i = 1:obj.nl
                mdh_all_matrix = mdh_struct_to_matrix(obj.mdh_all, 1);
                b_T_i = T_mdh_multi(mdh_all_matrix(1:i,:));
                base_p_link_i = b_T_i(1:3,4);
                base_R_link_i = b_T_i(1:3,1:3);
                obj.list_links(i).update(base_p_link_i,base_R_link_i);
                % Link.update
                obj.list_links(i).update_w_T_Link(obj.w_T_base*b_T_i);
                obj.list_links(i).update_w_T_Link_inhand(obj.w_T_base_inhand*b_T_i);
            end
            
            obj.base.update_w_T_Link(obj.w_T_base);
            obj.base.update_w_T_Link_inhand(obj.w_T_base_inhand);
            
            % update viapoints
            obj.update_viapoints;
            % update joints info
%             obj.update_joints_info;
            % update the list of tendons
            obj.update_M_coupling(obj.q_a); 
            
            % update the obstacle
            obj.update_obstacles;
            % update tendon force limits
%             obj.update_tendon_force_limits;
            % update dynamic parameters
%             obj.update_finger_par_dyn;    
            % update rsi model
%             obj.update_rst_model;
        end
        
        %% joint related functinos
        function update_joints_info(obj)
            % update joint information
            for i = 1:obj.nj
               obj.limits_q(i,1:2) = obj.list_joints(i).q_limits;
               obj.limits_qd(i,1:2) = obj.list_joints(i).qd_limits;
               obj.limits_qdd(i,1:2) = obj.list_joints(i).qdd_limits;
            end 
        end

        function set_onejoint_limits_q(obj, joint_index, q_limits)
            % update joint information
            obj.list_joints(joint_index).set_limit_q(q_limits);
            obj.update_joints_info;
        end
        function set_onejoint_limits_qd(obj, joint_index, qd_limits)
            % update joint information
            obj.list_joints(joint_index).set_limit_qd(qd_limits);
            obj.update_joints_info;
        end
        function set_onejoint_limits_qdd(obj, joint_index, qdd_limits)
            % update joint information
            obj.list_joints(joint_index).set_limit_qdd(qdd_limits);
            obj.update_joints_info;
        end

        function set_joint_limits_on(obj)
            % update joint information
            obj.limit_joint_on = 1;
        end
        function set_joint_limits_off(obj)
            % update joint information
            obj.limit_joint_on = 0;
        end
        
        function [q_sat, qd_sat, qdd_sat] = check_joints_limits(obj, q, qd, qdd)
            assert(length(q)== obj.nja, 'dimension of joint vector is incorrect!')
            assert(length(qd)== obj.nja, 'dimension of joint vector is incorrect!')
            assert(length(qdd)== obj.nja, 'dimension of joint vector is incorrect!')
            q_sat = q;
            q_sat(q < obj.limits_q(:,1)) = obj.limits_q(q < obj.limits_q(:,1),1);
            q_sat(q > obj.limits_q(:,2)) = obj.limits_q(q > obj.limits_q(:,2),2);
            qd_sat = qd;
            qd_sat(qd < obj.limits_q(:,3)) = obj.limits_q(qd < obj.limits_q(:,3),3);
            qd_sat(qd > obj.limits_q(:,4)) = obj.limits_q(qd > obj.limits_q(:,4),4);
            qdd_sat = qdd;
            qdd_sat(qdd < obj.limits_q(:,5)) = obj.limits_q(qdd < obj.limits_q(:,5),5);
            qdd_sat(qdd > obj.limits_q(:,6)) = obj.limits_q(qdd > obj.limits_q(:,6),6);
        end

        %% dynamic parameters
        function update_finger_par_dyn(obj)
            % update the finger dynamic parameters (only links, leave the
            % base parameter unchanged
            % TODO: finalize for all types
            
            mass_all = zeros(obj.nja+1,1); % first is base
            com_all = zeros(3,obj.nja+1);
            inertia_all = zeros(6,obj.nja+1);
            % set base information
            mass_all(1) = obj.base.par_dyn.mass;
            com_all(:,1) = obj.base.par_dyn.com;
            inertia_all(:,1) = obj.base.par_dyn.inertia;
            for i = 1:obj.nl
                mass_all(i+1) = obj.list_links(i).par_dyn.mass;
                com_all(:,i+1) = obj.list_links(i).par_dyn.com;
                inertia_all(:,i+1) = obj.list_links(i).par_dyn.inertia;
            end
            obj.par_dyn_f.mass_all = mass_all;
            obj.par_dyn_f.com_all = com_all;
            obj.par_dyn_f.inertia_all = inertia_all;
        end

        function set_g_w(obj,g)
            % set dynamic parameter in world frame
            % g is in world frame as input 
            g_reshape = reshape(g,3,1);
%             w_R_b = obj.w_R_base;
%             obj.par_dyn_f.g = w_R_b'*g_reshape;
            obj.par_dyn_f.g = g_reshape;
        end

        function set_base_dynpar(obj,mass,com,inertia )
            % NOT USED
            % to modify the base parameter, change the obj.base properties
            % directly
            % set dynamic parameter in world frame
            assert(length(mass) == 1, '[set_base_dynpar] mass dimentino is not correct')
            assert(length(com) == 3, '[set_base_dynpar] com dimentino is not correct')
            assert(length(inertia) == 6, '[set_base_dynpar] inertia dimentino is not correct')
            %  update the properties of Finger.base
            obj.base.par_dyn.mass = mass;
            obj.base.par_dyn.com = reshape(com,3,1);
            obj.base.par_dyn.inertia = reshape(inertia,6,1);
            % update Finger.par_dyn
            obj.par_dyn_f.mass_all(1) = mass;
            obj.par_dyn_f.com_all(:,1) = reshape(com,3,1) ; % regarding com
            obj.par_dyn_f.inertia_all(:,1) = reshape(inertia,6,1) ; 
            
        end

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%               

%         function rst_model_tmp = update_rst_model(obj)
%             % build/update the rst model of the finger based on the mdh
%             % parameters
%             %
%                          
%             mdh_matrix = mdh_struct_to_matrix(obj.mdh_ori,2); % mdh order: a,alpha,d,theta
% %             mdh_matrix_order_1 = mdh_struct_to_matrix(obj.mdh_ori,1);
%             % create rigid body tree
%             rst_model_tmp = rigidBodyTree;
%             rst_model_tmp.DataFormat = 'column';
%             rst_model_tmp.Gravity = obj.par_dyn_f.g;
%             
%             % convert inertia format to rst format
%             
%             inertia_all = obj.par_dyn_f.inertia_all;
%             mass_all = obj.par_dyn_f.mass_all;
%             com_all = obj.par_dyn_f.com_all;
%             
%             inertia_all_rst = zeros(size(obj.par_dyn_f.inertia_all)); % [xx yy zz yz xz xy]
%             for i = 1:obj.nj
%                 % transfer inertia into bodyframe
%                 inertia_all_rst(:,i+1) = inertia_all(:,i+1) + ...
%                     mass_all(i+1)*[com_all(2,i+1).^2+com_all(3,i+1).^2;com_all(1,i+1).^2+com_all(3,i+1).^2;com_all(1,i+1).^2+com_all(2,i+1).^2;...
%                     -com_all(2,i+1)*com_all(3,i+1);-com_all(1,i+1)*com_all(3,i+1);-com_all(1,i+1)*com_all(2,i+1)];
%             end
% 
%             % virtual first body:  from CS.base to CS.1
%             bodyname = obj.list_links(1).name;
%             body1 = rigidBody(bodyname);
%             jointname = strcat(obj.name,'_virtual_Base_joints');
%             jnt1 = rigidBodyJoint(jointname,'revolute');          
%             W_T_base = obj.get_W_T_B(); % World to Base 
% %             T = T_mdh_multi(mdh_matrix_order_1(1,:));
%             setFixedTransform(jnt1,W_T_base);
%             body1.Joint = jnt1;
%             body1.Mass = mass_all(2); % first body (virtual)
%             body1.Inertia = inertia_all_rst(:,2);
%             body1.CenterOfMass = com_all(:,2);          
%             addBody(rst_model_tmp,body1,'base');
%             bodyname_last = bodyname;
% %             bodyname_last = 'base';
%             for j = 2:obj.nl
%                 bodyname = obj.list_links(j).name;
%                 body1 = rigidBody(bodyname);
%                 jointname = strcat(obj.name,'_',obj.list_joints(j).name);
%                 jnt1 = rigidBodyJoint(jointname,'revolute');
%                 setFixedTransform(jnt1,mdh_matrix(j,:),'mdh');
%                 body1.Joint = jnt1;
%                 body1.Mass = mass_all(j+1); % first body (virtual)
%                 body1.Inertia = inertia_all_rst(:,j+1);
%                 body1.CenterOfMass = com_all(:,j+1);
%                 addBody(rst_model_tmp,body1,bodyname_last);
%                 bodyname_last = bodyname;
%             end
%             
%             % virtual last body:  fingertip
%             bodyname = strcat(obj.name,'_endeffector');
%             body1 = rigidBody(bodyname);
%             jointname = strcat(obj.name,'Virtual_fingertip');
%             jnt1 = rigidBodyJoint(jointname,'fixed');          
%             setFixedTransform(jnt1,mdh_matrix(end,:),'mdh');
%             body1.Joint = jnt1;
%             body1.Mass = 0;
%             body1.Inertia = [0,0,0,0,0,0];
%             body1.CenterOfMass = [0,0,0];
%             addBody(rst_model_tmp,body1,bodyname_last);
%             obj.rst_model = rst_model_tmp;
%         end
        
        function rst_model_tmp = update_rst_model(obj)
            % new version of rst model with a vitural base body
            % build/update the rst model of the finger based on the mdh
            % parameters
            %
                         
            mdh_matrix = mdh_struct_to_matrix(obj.mdh_ori,2); % mdh order: a,alpha,d,theta
%             mdh_matrix_order_1 = mdh_struct_to_matrix(obj.mdh_ori,1);
            % create rigid body tree
            rst_model_tmp = rigidBodyTree;
            rst_model_tmp.DataFormat = 'column';
            rst_model_tmp.Gravity = obj.par_dyn_f.g;
            
            % convert inertia format to rst format
            
            inertia_all = obj.par_dyn_f.inertia_all;
            mass_all = obj.par_dyn_f.mass_all;
            com_all = obj.par_dyn_f.com_all;
            
            inertia_all_rst = zeros(size(obj.par_dyn_f.inertia_all)); % [xx yy zz yz xz xy]
            for i = 0:obj.nj
                % transfer inertia into bodyframe
                inertia_all_rst(:,i+1) = inertia_all(:,i+1) + ...
                    mass_all(i+1)*[com_all(2,i+1).^2+com_all(3,i+1).^2;com_all(1,i+1).^2+com_all(3,i+1).^2;com_all(1,i+1).^2+com_all(2,i+1).^2;...
                    -com_all(2,i+1)*com_all(3,i+1);-com_all(1,i+1)*com_all(3,i+1);-com_all(1,i+1)*com_all(2,i+1)];
            end

            % virtual first body:  from CS.base to CS.1
            bodyname = strcat(obj.name,'_virtual_base');
            body1 = rigidBody(bodyname);
            jointname = strcat(obj.name,'_w_to_base');
            jnt1 = rigidBodyJoint(jointname,'fixed');          
            W_T_base = obj.get_W_T_B(); % World to Base 
%             T = T_mdh_multi(mdh_matrix_order_1(1,:));
            setFixedTransform(jnt1,W_T_base);
            body1.Joint = jnt1;
            body1.Mass = mass_all(1); % first body (virtual)
            body1.Inertia = inertia_all_rst(:,1);
            body1.CenterOfMass = com_all(:,1);          
            addBody(rst_model_tmp,body1,'base');
            bodyname_last = bodyname;
%             bodyname_last = 'base';
            for j = 1:obj.nl
                bodyname = obj.list_links(j).name;
                body1 = rigidBody(bodyname);
                jointname = strcat(obj.name,'_',obj.list_joints(j).name);
                jnt1 = rigidBodyJoint(jointname,'revolute');
                setFixedTransform(jnt1,mdh_matrix(j,:),'mdh');
                body1.Joint = jnt1;
                body1.Mass = mass_all(j+1); % first body (virtual)
                body1.Inertia = inertia_all_rst(:,j+1);
                body1.CenterOfMass = com_all(:,j+1);
                addBody(rst_model_tmp,body1,bodyname_last);
                bodyname_last = bodyname;
            end
            
            % virtual last body:  fingertip
            bodyname = strcat(obj.name,'_endeffector');
            body1 = rigidBody(bodyname);
            jointname = strcat(obj.name,'Virtual_fingertip');
            jnt1 = rigidBodyJoint(jointname,'fixed');          
            setFixedTransform(jnt1,mdh_matrix(end,:),'mdh');
            body1.Joint = jnt1;
            body1.Mass = 0;
            body1.Inertia = [0,0,0,0,0,0];
            body1.CenterOfMass = [0,0,0];
            addBody(rst_model_tmp,body1,bodyname_last);
            obj.rst_model = rst_model_tmp;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % calculate kinematics 
        function set_base(obj, w_p_base, w_R_base)
            % set the position and orientation of the base in the world
            % frame
            obj.w_p_base = reshape(w_p_base,3,1);
            obj.w_R_base = w_R_base;

            obj.w_T_base = pR2T(w_p_base,w_R_base);
            obj.w_T_base_inhand = obj.w_T_base;

            obj.update_finger(obj.q_a);
        end
        function set_base_p(obj, w_p_base)
            % set the position and orientation of the base in the world
            % frame
            obj.w_p_base = reshape(w_p_base,3,1);
            obj.w_T_base = pR2T(w_p_base,obj.w_R_base);
            obj.w_T_base_inhand = obj.w_T_base;

            obj.update_finger(obj.q_a);
        end
        function set_base_R(obj, w_R_base)
            % set the position and orientation of the base in the world
            % frame
            obj.w_R_base = w_R_base;
            obj.w_T_base = pR2T(obj.w_p_base,w_R_base);
            obj.w_T_base_inhand = obj.w_T_base;

            obj.update_finger(obj.q_a);
        end

        function [w_p_base,w_R_base] = get_base(obj)
            % set the position and orientation of the base in the world
            % frame
            w_p_base = obj.w_p_base;
            w_R_base = obj.w_R_base;
        end
        function w_p_base = get_w_p_b(obj)
            % set the position and orientation of the base in the world
            % frame
            w_p_base = obj.w_p_base;
        end
        function w_R_base = get_w_R_b(obj)
            % set the position and orientation of the base in the world
            % frame
            w_R_base = obj.w_R_base;
        end


        function W_T_b = get_W_T_B(obj)
            % get the homogeneous transformation matrix from Base to W
            W_T_b = obj.w_T_base;
        end

        function [p_link_all_w,p_link_all_b] = get_p_all_links(obj)
            % get the Cartesian position of the base and link ends
            w_R_b = obj.w_R_base; 
            w_p_b = obj.w_p_base;
%             w_T_b = get_W_T_B(obj);
            p_link_all_b = zeros(3,obj.nl+1);
            p_link_all_w = zeros(3,obj.nl+1);

%             p_link_all_b = sym(zeros(3,obj.nl+1));
%             p_link_all_w = sym(zeros(3,obj.nl+1));
            for i = 1:obj.nl
                p_link_all_b(:,i) = obj.list_links(i).base_p;
                p_link_all_w(:,i) = w_R_b * p_link_all_b(:,i) + w_p_b;
            end
            % fingertip position
            mdh_all_matrix = mdh_struct_to_matrix(obj.mdh_all, 1);
            b_T_i = T_mdh_multi(mdh_all_matrix);
            p_link_all_b(:,end) = b_T_i(1:3,4);
            p_link_all_w(:,end) = w_p_b + w_R_b * b_T_i(1:3,4);
        end

%         function w_T_all = get_T_all_links(obj)
%             % get all transformation matrix of all links
%             w_T_b = obj.get_W_T_B();
%             w_T_all = zeros(4,4,obj.nl+2); % base, link1, ... linkn, ee
%             w_T_all(:,:,1) = w_T_b;
%             for i = 1:obj.nl
%                 b_p_i = obj.list_links(i).base_p;
%                 b_R_i = obj.list_links(i).base_R;
%                 b_T_i = [b_R_i,b_p_i; 0 0 0 1];
%                 w_T_all(:,:,i+1) = w_T_b*b_T_i;
%             end
%             % fingertip position
%             mdh_all_matrix = mdh_struct_to_matrix(obj.mdh_all, 1);
%             b_T_ee = T_mdh_multi(mdh_all_matrix);
%             w_T_all(:,:,end) = w_T_b*b_T_ee;
%         end

        function w_T_all = get_T_all_links(obj)
            % get all transformation matrix of all links
            w_T_b = obj.get_W_T_B();
            w_T_all = zeros(4,4,obj.nl+2); % base, link1, ... linkn, ee
            w_T_all(:,:,1) = w_T_b;
            for i = 1:obj.nl
                w_T_all(:,:,i+1) = obj.list_links(i).w_T_Link;
            end
            % fingertip position
            mdh_all_matrix = mdh_struct_to_matrix(obj.mdh_all, 1);
            b_T_ee = T_mdh_multi(mdh_all_matrix);
            w_T_all(:,:,end) = w_T_b*b_T_ee;
        end

        function w_T_all = get_T_all_links_inhand(obj)
            % get all transformation matrix of all links
            % first matrix is base, last is endeffector
            w_T_b = obj.get_w_T_base_inhand();
            w_T_all = zeros(4,4,obj.nl+2); % base, link1, ... linkn, ee
            w_T_all(:,:,1) = w_T_b;
            for i = 1:obj.nl
                b_p_i = obj.list_links(i).base_p;
                b_R_i = obj.list_links(i).base_R;
                b_T_i = [b_R_i,b_p_i; 0 0 0 1];
                w_T_all(:,:,i+1) = w_T_b*b_T_i;
            end
            % fingertip position
            mdh_all_matrix = mdh_struct_to_matrix(obj.mdh_all, 1);
            b_T_ee = T_mdh_multi(mdh_all_matrix);
            w_T_all(:,:,end) = w_T_b*b_T_ee;
        end
        
        function [w_p_ee,w_R_ee, w_T_ee] = get_T_ee_w(obj)
            % get the Cartesian position of the base and link ends
            w_R_b = obj.w_R_base; 
            w_p_b = obj.w_p_base;
%             w_T_b = get_W_T_B(obj);
%             w_R_ee = zeros(3,3);
%             w_p_ee = zeros(3,1);
            % fingertip position
            mdh_all_matrix = mdh_struct_to_matrix(obj.mdh_all, 1);
            b_T_i = T_mdh_multi(mdh_all_matrix);
            w_p_ee = w_p_b + w_R_b * b_T_i(1:3,4);
            w_R_ee =  w_R_b * b_T_i(1:3,1:3);
            w_T_ee = [w_R_ee,w_p_ee;0,0,0,1];
        end

        % as a finger in the hand object (use w_T_base_inhand instead w_T_base)
        function set_w_T_base_inhand(obj, w_T_base_inhand)
            % set the position and orientation of the base in the world
            % frame in a hand object
            obj.w_T_base_inhand = w_T_base_inhand;
        end

        function w_T_base_inhand = get_w_T_base_inhand(obj)
            % get the homogeneous transformation matrix from Base to W 
            % in a hand object
            w_T_base_inhand = obj.w_T_base_inhand;
        end

        function w_T_ee_inhand = get_w_T_ee_inhand(obj)
            % get the Cartesian position of the base and link ends
%             w_T_base_inhand = obj.get_w_T_base_inhand; 
            % fingertip position
            mdh_all_matrix = mdh_struct_to_matrix(obj.mdh_all, 1);
            b_T_i = T_mdh_multi(mdh_all_matrix);
            w_T_ee_inhand = obj.w_T_base_inhand*b_T_i;
        end

        function x = get_x_ee_w(obj)
            % get the Cartesian position of link ends
            [w_p_ee,w_R_ee] = obj.get_T_ee_w();
            x = [w_p_ee;R2euler_XYZ(w_R_ee)];
        end

        %% contacts

        function contact_obj = add_contact(obj, name, link_index, link_p_obj)
            % add contact to the specific link 
            contact_obj = obj.list_links(link_index).add_contact_link(name, link_p_obj);
            obj.update_list_contacts();
        end
        
        function update_list_contacts(obj)
            % update the Link Class property: Links.nc 
            % TODO: change name of the function
            %       update all link contact (optional)
            obj.list_contacts = []; % init list_contacts
            num_contact = 0;
            if obj.nl 
                for i = 1:obj.nl
                    num_contact_link_i =  obj.list_links(i).nc;
                    num_contact = num_contact + num_contact_link_i;
                    if num_contact_link_i ~= 0
                        for j = 1:num_contact_link_i
                            obj.list_contacts = [obj.list_contacts;obj.list_links(i).contacts(j)];
                        end
                    end
                end
            end
            obj.nc = num_contact;
        end
        
        function [w_p_contacts_all,b_p_contacts_all] = get_p_all_contacts(obj)
            % plot 3d contact points
            w_R_b = obj.w_R_base;
            w_p_b = obj.w_p_base;
            %             w_T_b = get_W_T_B(obj);
            b_p_contacts_all = zeros(3,obj.nc);
            w_p_contacts_all = zeros(3,obj.nc);
            for i = 1:obj.nc
                contact_pos_i = obj.list_contacts(i).base_p;
                w_contact_pos = w_R_b * contact_pos_i + w_p_b;
                b_p_contacts_all(:,i) = contact_pos_i;
                w_p_contacts_all(:,i) = w_contact_pos;
            end
        end

        function delete_all_contacts(obj)
            % delete all contacts of the finger
            if obj.nl 
                for i = 1:obj.nl
                    obj.list_links(i).delete_all_contacts_link();
                end
            end
            obj.update_list_contacts;
        end


        %% Visualization
%         function print_finger(obj,varargin) % not used 
%             % plot 3d contact points 
%             % [06.09.2023] new version of plot is avaible: plot_finger
%             if nargin == 1
%                 color = 'r';
%                 linewidth = 3;
%                 markersize = 5;
%             elseif  nargin == 2
%                 color = varargin{1};
%                 linewidth = 3;
%                 markersize = 5;
%             elseif  nargin == 3
%                 color = varargin{1};
%                 linewidth = varargin{2};
%                 markersize = 5;
%             elseif  nargin == 4
%                 color = varargin{1};
%                 linewidth = varargin{2};
%                 markersize = varargin{3};
%             end
%             p_link_all_w_r = obj.get_p_all_links;
%             plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color',color,...
%                                     'LineWidth',linewidth,'MarkerSize',markersize);
%             hold on
%         end

        function plot_finger(obj,varargin) 
            % [06.09.2023] plot the finger with paramter structures
            % parameters
            %           .inhand = 0
            %           .linecolor = 'r'
            %           .linewidth = 3
            %           .markersize = 5
            %           .axis_show = 1
            %           .axis_length = 0.1
            %           .linewidth = 3
            %           .linewidth = 3
            
            if nargin == 1
                parameters = obj.plot_parameter_init;
            elseif nargin == 2
                parameters =  varargin{1};
            else
                error('[plot_finger] input dimension is incorrect! \n')
            end
            if parameters.inhand == 0
                w_R_b = obj.w_R_base;
                w_p_b = obj.w_p_base;
            else
                [w_p_b,w_R_b] = T2pR(obj.get_w_T_base_inhand);
            end

            color = parameters.linecolor;
            linewidth = parameters.linewidth;
            markersize = parameters.markersize;

            [~,p_link_all_b_r] = obj.get_p_all_links;
            p_link_all_w_r = w_p_b + w_R_b * p_link_all_b_r;
            plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color',color,...
                                    'LineWidth',linewidth,'MarkerSize',markersize);
            hold on
            if parameters.axis_show
                for i = 1:obj.nl
%                     length_link_i = obj.list_links(i).Length;
                    length_link_i = parameters.axis_len;
                    W_p_link_i = w_p_b + w_R_b*obj.list_links(i).base_p;
                    W_p_link_z_i = 0.5*length_link_i*w_R_b*obj.list_links(i).base_R*[0;0;1];
                    h = quiver3(W_p_link_i(1),W_p_link_i(2),W_p_link_i(3),...
                            W_p_link_z_i(1),W_p_link_z_i(2),W_p_link_z_i(3),...
                            'Color','b','LineWidth',linewidth);
                    set(h,'AutoScale','on', 'AutoScaleFactor',1);
                    hold on
                    xlabel('x')
                    ylabel('y')
                    zlabel('z')
                end 
            end
        end

        function parstr = plot_parameter_init(obj)
            % init the parameter struct for plot_finger
            % check plot_parameter_init.m
            parstr = plot_parameter_init();
        end

        function plot_com(obj,varargin) 
            % plot the center of mass of each link in world frame
            
            if nargin == 1
                par = plot_parameter_init();
            elseif nargin == 2
                par =  varargin{1};
            else
                error('[plot_com] input dimension is incorrect! \n')
            end
            if par.inhand == 0
                w_R_b = obj.w_R_base;
                w_p_b = obj.w_p_base;
            else
                [w_p_b,w_R_b] = T2pR(obj.get_w_T_base_inhand);
            end
%             p_link_all_b = zeros(3,obj.nl+1);
            p_com_all_b = obj.par_dyn_f.com_all;
            p_com_all_w = zeros(3,obj.nl+1);
            for i = 1:obj.nl
                b_p_link = obj.list_links(i).base_p;
                b_R_link = obj.list_links(i).base_R;
                p_com_all_w(:,i+1) = w_R_b * (b_p_link+b_R_link*p_com_all_b(:,i+1)) + w_p_b;
                
            end
            p_com_all_w(:,1) = w_R_b * (obj.par_dyn_f.com_all(:,1)) + w_p_b;
            plot3(p_com_all_w(1,:)',p_com_all_w(2,:)',p_com_all_w(3,:)',par.com_marker, ...
                'Color',par.com_markercolor, 'MarkerSize',par.com_markersize);
            hold on
            % fingertip position
%             mdh_all_matrix = mdh_struct_to_matrix(obj.mdh_all, 1);
%             b_T_i = T_mdh_multi(mdh_all_matrix);
%             p_link_all_b(:,end) = b_T_i(1:3,4);
%             p_link_all_w(:,end) = w_p_b + w_R_b * b_T_i(1:3,4);
        end

        function plot_contacts(obj,varargin) 
            % plot the com of each link in world frame
            
            if nargin == 1
                par = obj.plot_parameter_init;
            elseif nargin == 2
                par =  varargin{1};
            else
                error('[plot_contact] input dimension is incorrect! \n')
            end
            [w_p_contacts_all,~] = obj.get_p_all_contacts;

            plot3(w_p_contacts_all(1,:)',w_p_contacts_all(2,:)',w_p_contacts_all(3,:)','*','Color',par.markercolor,...
                                    'MarkerSize',par.markersize);
            hold on
            % fingertip position
%             mdh_all_matrix = mdh_struct_to_matrix(obj.mdh_all, 1);
%             b_T_i = T_mdh_multi(mdh_all_matrix);
%             p_link_all_b(:,end) = b_T_i(1:3,4);
%             p_link_all_w(:,end) = w_p_b + w_R_b * b_T_i(1:3,4);
        end

        function plot_viapoints(obj,varargin) 
            % plot the com of each link in world frame
            
            if nargin == 1
                par = obj.plot_parameter_init;
            elseif nargin == 2
                par =  varargin{1};
            else
                error('[Finger:plot_viapoints] input dimension is incorrect! \n')
            end
            
            if obj.nvia == 0
                fprintf('[Finger:plot_viapoints]: Finger %s has no via point! \n', obj.name)
            else
                for i = 1:obj.nvia
                    vp_i = obj.list_viapoints(i);
                    vp_i.plot_viapoints(par);
                end
            end
%             if par.inhand == 0
%                 w_R_b = obj.w_R_base;
%                 w_p_b = obj.w_p_base;
%             else
%                 [w_p_b,w_R_b] = T2pR(obj.get_w_T_base_inhand);
%             end
% 
%             [~,b_p_viapoints_all] = obj.get_p_all_viapoints;
%             w_p_viapoints_all = w_p_b + w_R_b*b_p_viapoints_all;
%             plot3(w_p_viapoints_all(1,:)',w_p_viapoints_all(2,:)',w_p_viapoints_all(3,:)',par.viapoint_marker,...
%                     'Color',par.viapoint_markercolor,'MarkerSize',par.viapoint_markersize);
%             hold on
        end

        function plot_muscles(obj,varargin) 
            % plot the com of each link in world frame
            
            if nargin == 1
                par = obj.plot_parameter_init;
            elseif nargin == 2
                par =  varargin{1};
            else
                error('[plot_muscles] input dimension is incorrect! \n')
            end

            for i = 1:obj.nmus
                mus_i = obj.list_muscles(i);
                mus_i.plot_muscles(par);
%                 w_p_viapoints_all = obj.get_p_muscle_viapoints(i);
%                 plot3(w_p_viapoints_all(1,:)',w_p_viapoints_all(2,:)',w_p_viapoints_all(3,:)','-.',...
%                     'Color',par.muscle_linecolor,'MarkerSize',par.muscle_markersize,'LineWidth',par.muscle_linewidth);
            end
        end
    
        function plot_obstacles(obj,varargin)
            % plot the com of each link in world frame

            if nargin == 1
                par = obj.plot_parameter_init;
            elseif nargin == 2
                par =  varargin{1};
            else
                error('[Finger:plot_obstacles] input dimension is incorrect! \n')
            end
            if obj.nobs ~= 0
                for i = 1:obj.nobs
                    obs_i = obj.list_obstacles{i};
                    obs_i.plot_obs(par);
                end
            end
        end
        % plot function for hand 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% tendon related:
        function add_tendon(obj, name, poly3_par)
            % add tendon to class
            % TODO: partition q & qa (current version: using q_a)
            
            assert(all(size(poly3_par)==[4,obj.nj]) == 1, 'tendon parameter for poly3 has wrong dimension !!');
            tendon_tmp = Tendons(name);
            tendon_tmp.init_tendon_par(obj.nj, obj.nt+1);
            tendon_tmp.set_par_MA_poly3(poly3_par,obj.q);
            obj.nt = obj.nt + 1;
            obj.list_tendons = [obj.list_tendons;tendon_tmp];
            obj.update_tendon_ma_limits_from_joints;
            obj.update_M_coupling(obj.q_a);
%             obj.update_tendon_force_limits;
        end

        function set_tendon_par_MA_poly3(obj, tendon_index, joint_index, par)
            
            assert(length(par)== 4, 'dimension of par_MA_poly3 is incorrect!')
            assert(tendon_index <= obj.nt, 'index of tendon is incorrect!')
            assert(joint_index <= obj.list_tendons(tendon_index).nj_finger, 'index of joint is incorrect!')
            par_MA_poly3_i = obj.list_tendons(tendon_index).get_par_MA_poly3;
            par_MA_poly3_i(:,joint_index) = par;
            obj.list_tendons(tendon_index).set_par_MA_poly3(par_MA_poly3_i, obj.q_a);
            obj.update_M_coupling(obj.q_a);
            
        end
        
        function update_tendon_ma_limits_from_joints(obj)
            obj.limits_t_ma = zeros(obj.nj,2);
            for i = 1:obj.nj
                obj.limits_t_ma(i,:) = obj.list_joints(i).momentarm_limits;
            end
        end
        
        function M_coupling = update_M_coupling(obj, q)
            % update M_coupling matrix w.r.t. current configuration
            % TODO: moment arm value as a function of q
            M_coupling_tmp = zeros(obj.nj,obj.nt);
            for i = 1:obj.nt
                obj.list_tendons(i).update_momentarm(q);
                M_coupling_tmp(:,i) = obj.list_tendons(i).ma_value;
            end
            obj.M_coupling = M_coupling_tmp;
            obj.moment_arm_limit;
            M_coupling = obj.M_coupling;
        end
        
        function moment_arm_limit(obj)
            % saturate the moment arm limits w.r.t. joint properties
            % called in update_M_coupling.m
            for i = 1:obj.nj
                ma_max_i = obj.list_joints(i).momentarm_limits(2);
                ma_min_i = obj.list_joints(i).momentarm_limits(1);
                M_coupling_i = obj.M_coupling(i,:);
                M_coupling_i(M_coupling_i>ma_max_i) = ma_max_i;
                M_coupling_i(M_coupling_i<ma_min_i) = ma_min_i;
                obj.M_coupling(i,:) = M_coupling_i;
            end
        end
        
        function update_tendon_force_limits(obj)
            % update tendon force limits
            %   1. limits_q
            for i = 1:obj.nt
               obj.limits_ft(i,:) = obj.list_tendons(i).f_limits;
            end 
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% ViaPoint related 

        function new_viapoint = add_ViaPoint(obj, name, link_index, link_p_obj)
            % add contact to the specific link 
            if link_index == 0
                link_name = obj.base.name;
                new_viapoint = obj.base.add_viapoint_link(strcat(link_name,'_',name), link_p_obj);
            else
                link_name = obj.list_links(link_index).name;
                new_viapoint = obj.list_links(link_index).add_viapoint_link(strcat(link_name,'_',name), link_p_obj);
            end
            obj.update_list_viapoints();
        end

        function update_list_viapoints(obj)
            % update the Link Class property: Links.nc 
            obj.list_viapoints = []; % init list_contacts
            num_viapoints = 0;
            % add viapoint from base
            num_vp_link_i =  obj.base.nv;
            num_viapoints = num_viapoints + num_vp_link_i;
            if num_vp_link_i ~= 0
                for j = 1:num_vp_link_i
                    obj.list_viapoints = [obj.list_viapoints;obj.base.viapoints(j)];
                end
            end
            % add viapoint from links
            if obj.nl 
                for i = 1:obj.nl
                    num_vp_link_i =  obj.list_links(i).nv;
                    num_viapoints = num_viapoints + num_vp_link_i;
                    if num_vp_link_i ~= 0
                        for j = 1:num_vp_link_i
                            obj.list_viapoints = [obj.list_viapoints;obj.list_links(i).viapoints(j)];
                        end
                    end
                end
            end
            obj.nvia = num_viapoints;
            obj.update_viapoints;
        end

        function update_viapoints(obj)
            % update position of all viapoints
            for i = 1:obj.nvia
                viapoint_i = obj.list_viapoints(i);
                w_p_viapoint = obj.get_p_viapoint(i);
                w_p_viapoint_inhand = obj.get_p_viapoint_inhand(i);
                viapoint_i.update_w_p_VP(w_p_viapoint);
                viapoint_i.update_w_p_VP_inhand(w_p_viapoint_inhand);
            end
        end
        
        function [w_p_viapoints_all,b_p_viapoints_all] = get_p_all_viapoints(obj)
            % plot 3d contact points
            w_R_b = obj.w_R_base;
            w_p_b = obj.w_p_base;
            %             w_T_b = get_W_T_B(obj);
            b_p_viapoints_all = zeros(3,obj.nvia);
            w_p_viapoints_all = zeros(3,obj.nvia);
            for i = 1:obj.nvia
                viapoint_pos_i = obj.list_viapoints(i).base_p;
                w_viapoint_pos = w_R_b * viapoint_pos_i + w_p_b;
                b_p_viapoints_all(:,i) = viapoint_pos_i;
                w_p_viapoints_all(:,i) = w_viapoint_pos;
            end
        end

        function [w_p_viapoint,b_p_viapoint] = get_p_viapoint(obj, VP_index)
            % 
            w_R_b = obj.w_R_base;
            w_p_b = obj.w_p_base;

            viapoint_pos_i = obj.list_viapoints(VP_index).base_p;
            w_viapoint_pos = w_R_b * viapoint_pos_i + w_p_b;
            b_p_viapoint = viapoint_pos_i;
            w_p_viapoint = w_viapoint_pos;
        end

        function [w_p_viapoints_all,b_p_viapoints_all] = get_p_all_viapoints_inhand(obj)
            % plot 3d viapoints
            w_T_b = obj.w_T_base_inhand;
            w_R_b = w_T_b(1:3,1:3);
            w_p_b = w_T_b(1:3,4);

            b_p_viapoints_all = zeros(3,obj.nvia);
            w_p_viapoints_all = zeros(3,obj.nvia);
            for i = 1:obj.nvia
                viapoint_pos_i = obj.list_viapoints(i).base_p;
                w_viapoint_pos = w_R_b * viapoint_pos_i + w_p_b;
                b_p_viapoints_all(:,i) = viapoint_pos_i;
                w_p_viapoints_all(:,i) = w_viapoint_pos;
            end
        end

        function [w_p_viapoint_inhand,b_p_viapoint_inhand] = get_p_viapoint_inhand(obj, VP_index)
            % plot 3d contact points
            w_T_b = obj.w_T_base_inhand;
            w_R_b = w_T_b(1:3,1:3);
            w_p_b = w_T_b(1:3,4);

            viapoint_pos_i = obj.list_viapoints(VP_index).base_p;
            w_viapoint_pos = w_R_b * viapoint_pos_i + w_p_b;
            b_p_viapoint_inhand = viapoint_pos_i;
            w_p_viapoint_inhand = w_viapoint_pos;
        end

%         function add_ViaPoint_to_muscle(obj, name, link_index, link_p_obj, muscle_index)
%             % add contact to the specific link 
%             assert(muscle_index <= obj.nmus, 'add_ViaPoint_to_muscle: muscle index exceeds the number of muscles!')
%             obj.list_links(link_index).add_viapoint_link(name, link_p_obj);
%             obj.list_muscles(muscle_index).list_vp = [obj.list_muscles(muscle_index).list_vp; viapoint_obj];
%             obj.list_muscles(muscle_index).update_viapoints; 
%             obj.update_list_viapoints();
%         end

        function delete_all_viapoint(obj)
            % delete all contacts of the finger
            if obj.nl 
                for i = 1:obj.nl
                    obj.list_links(i).delete_all_contacts_link();
                end
            end
            obj.update_list_contacts;
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Muscle related 
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

        function update_muscles(obj)
            % update position of all viapoints
            for i = 1:obj.nmus
                muscle_i = obj.list_muscles(i);
                muscle_i.init_list_constr;
            end
        end

        function [w_p_viapoints_all,b_p_viapoints_all] = get_p_muscle_viapoints(obj, muscle_index)
            assert(isempty(find(muscle_index - obj.nmus > 0)),'[get_p_muscle_viapoints] muscle index exceeds the number of muscles!')
            % plot 3d muscle via points
            w_R_b = obj.w_R_base;
            w_p_b = obj.w_p_base;
            n_mus_sel = length(muscle_index);
            n_vp = 0;
            for i = 1:n_mus_sel
                n_vp = n_vp + obj.list_muscles(muscle_index(i)).n_vp;
            end
            b_p_viapoints_all = zeros(3*n_mus_sel,n_vp);
            w_p_viapoints_all = zeros(3*n_mus_sel,n_vp);
            for i = 1:n_mus_sel
                muscle_i = obj.list_muscles(muscle_index(i));
                if muscle_i.n_vp == 0
                    fprintf('[get_p_muscle_viapoints] muscle %d has no viapoints! \n',i)
                end
                for j = 1:muscle_i.n_vp
                    viapoint_pos_i = muscle_i.list_vp(j).base_p;
                    w_viapoint_pos = w_R_b * viapoint_pos_i + w_p_b;
                    b_p_viapoints_all(3*i-2:3*i,j) = viapoint_pos_i;
                    w_p_viapoints_all(3*i-2:3*i,j) = w_viapoint_pos;
                end
            end
        end

        function Muscle_length = cal_all_Muscle_Length(obj)
            % calculate the length of all muscles
            Muscle_length = zeros(obj.nmus,1);
            for i = 1: obj.nmus
                Muscle_length(i) = obj.list_muscles(i).cal_muscle_length; 
            end
        end

        function Muscle_length = cal_selected_Muscle_Length(obj, muscle_index)
            % calculate the length of selected muscles
            assert(isempty(find(muscle_index - obj.nmus > 0)),'[cal_selected_Muscle_Length] muscle index exceeds the number of muscles!')
           
            Muscle_length = zeros(length(muscle_index),1);
            for i = 1: length(muscle_index)
                Muscle_length(i) = obj.list_muscles(muscle_index(i)).cal_muscle_length; 
            end
        end

        function MA = cal_all_Muscle_momentarm_1st_c(obj, step)
            % calculate the Momentarm matrix of all muscles
            % MA is a [nj,nmus] matrix
            % calculate the MA value using first order centered 
            % numeric differentiation method
            MA = zeros(obj.nj,obj.nmus);
            q_ori = obj.q;
            for i = 1: obj.nj
                for j = 1: obj.nmus
                    q_f = q_ori;
                    q_f(i) = q_f(i) + step;
                    obj.update_finger(q_f);
                    l_f = obj.cal_selected_Muscle_Length(j); % forward 
                    q_b = q_ori;
                    q_b(i) = q_b(i) - step;
                    obj.update_finger(q_b);
                    l_b = obj.cal_selected_Muscle_Length(j); % forward 
                    MA(i,j) = (l_f-l_b)/(2*step); 
                end
            end
            obj.update_finger(q_ori);
        end

        function MA = cal_selected_Muscle_momentarm_1st_c(obj, step, muscle_index)
            % calculate the Momentarm matrix of all muscles
            % MA is a [nj,length(muscle_index)] matrix
            % calculate the MA value using first order centered 
            % numeric differentiation method
            assert(isempty(find(muscle_index - obj.nmus > 0)),'[cal_selected_Muscle_momentarm_1st_c] muscle index exceeds the number of muscles!')
            MA = zeros(obj.nj,length(muscle_index));
            q_ori = obj.q;
            for i = 1: obj.nj
                for j = 1: length(muscle_index)
                    q_f = q_ori;
                    q_f(i) = q_f(i) + step;
                    obj.update_finger(q_f);
                    l_f = obj.cal_selected_Muscle_Length(muscle_index(j)); % forward 
                    q_b = q_ori;
                    q_b(i) = q_b(i) - step;
                    obj.update_finger(q_b);
                    l_b = obj.cal_selected_Muscle_Length(muscle_index(j)); % forward 
                    MA(i,j) = (l_f-l_b)/(2*step); 
                end
            end
            obj.update_finger(q_ori);
        end

        %% Obstacles
        function new_obstacle = add_Obstacle(obj, name, link_index, link_p_obj,link_R_obj)
            % add contact to the specific link 
            if link_index == 0
                link_name = obj.base.name;
                new_obstacle = obj.base.add_obstacle_link(strcat(link_name,'_',name), link_p_obj,link_R_obj);
            else
                link_name = obj.list_links(link_index).name;
                new_obstacle = obj.list_links(link_index).add_obstacle_link(strcat(link_name,'_',name), ...
                    link_p_obj,link_R_obj);
            end
            obj.update_list_obstacles();
        end
        

        function new_obstacle = add_Obstacle_cylinder(obj, name, link_index, link_p_obj,link_R_obj, radius, height)
            % add contact to the specific link 
            if link_index == 0
                link_name = obj.base.name;
                new_obstacle = obj.base.add_obstacle_cyl_link(strcat(link_name,'_',name), link_p_obj,link_R_obj);
            else
                link_name = obj.list_links(link_index).name;
                new_obstacle = obj.list_links(link_index).add_obstacle_cyl_link(strcat(link_name,'_',name), ...
                    link_p_obj,link_R_obj);
            end
            new_obstacle.radius = radius;
            new_obstacle.height = height;
            obj.update_list_obstacles();
        end


        function update_list_obstacles(obj)
            % update the Link Class property: Links.nc 
            obj.list_obstacles = {}; % init list_contacts
            num_obstacles = 0;
            % add viapoint from base
            num_obs_link_i =  obj.base.nobs;
            num_obstacles = num_obstacles + num_obs_link_i;
            if num_obstacles ~= 0
                for j = 1:num_obstacles
                    obj.list_obstacles = {obj.list_obstacles{:},obj.base.obstacles(j)}';
                end
            end
            % add viapoint from links
            if obj.nl 
                for i = 1:obj.nl
                    num_obs_link_i =  obj.list_links(i).nobs;
                    num_obstacles = num_obstacles + num_obs_link_i;
                    if num_obs_link_i ~= 0
                        for j = 1:num_obs_link_i
                            obj.list_obstacles = {obj.list_obstacles{:},obj.list_links(i).obstacles(j)}';
                        end
                    end
                end
            end
            obj.nobs = num_obstacles;
            
        end
        function update_obstacles(obj)
            % update transformation matrix of all obstacles
            if obj.nobs ~= 0
                for i = 1:obj.nobs
                    obstacle_i = obj.list_obstacles{i};
                    base_T_obs_i = obstacle_i.get_base_T_obs;
                    obstacle_i.update_w_T_Obs(obj.w_T_base*base_T_obs_i);
                    obstacle_i.update_w_T_Obs_inhand(obj.w_T_base_inhand * base_T_obs_i);
                end
            end
        end

        function [w_p_obs,b_p_obs] = get_p_obstacle(obj, obs_index)
            % get translational vector of one obstacle
            w_T_Obs_i = obj.list_obstacles{obs_index}.w_T_Obs;
            base_T_obs_i = obj.list_obstacles{obs_index}.base_T_obs;
            b_p_obs = base_T_obs_i(1:3,4);
            w_p_obs = w_T_Obs_i(1:3,4);
        end

        function [w_p_obs,b_p_obs] = get_p_all_obstacle(obj)
            % get translational vector of all obstacles
            w_p_obs = zeros(3, obj.nvia);
            b_p_obs = zeros(3, obj.nvia);
            for i = 1:obj.nvia
                w_T_Obs_i = obj.list_obstacles{i}.w_T_Obs;
                base_T_obs_i = obj.list_obstacles{i}.base_T_obs;
                b_p_obs(1:3,i) = base_T_obs_i(1:3,4);
                w_p_obs(1:3,i) = w_T_Obs_i(1:3,4);
            end
        end
        function [w_p_obs,b_p_obs] = get_p_all_obstacle_inhand(obj)
            % get translational vector of all obstacles
            w_p_obs = zeros(3, obj.nvia);
            b_p_obs = zeros(3, obj.nvia);
            for i = 1:obj.nvia
                w_T_Obs_i = obj.list_obstacles{i}.w_T_Obs_inhand;
                base_T_obs_i = obj.list_obstacles{i}.base_T_obs;
                b_p_obs(1:3,i) = base_T_obs_i(1:3,4);
                w_p_obs(1:3,i) = w_T_Obs_i(1:3,4);
            end
        end

        function [w_T_obs,b_T_obs] = get_T_obstacle(obj, obs_index)
            % get translational vector of one obstacle
            w_T_obs = obj.list_obstacles{obs_index}.w_T_Obs;
            b_T_obs = obj.list_obstacles{obs_index}.base_T_obs;
        end
        function [w_T_obs,b_T_obs] = get_T_obstacle_inhand(obj, obs_index)
            % get translational vector of one obstacle
            w_T_obs = obj.list_obstacles{obs_index}.w_T_Obs_inhand;
            b_T_obs = obj.list_obstacles{obs_index}.base_T_obs;
        end
    end
end

