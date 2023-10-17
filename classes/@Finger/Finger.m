%% Class of Finger:
% 
%       Create a Finger:
%           finger = Finger(finger_name, finger_type, link_len_vector)
%           
%           finger_name: [char]           
%           finger_type: 
%               'RRRR': general finger with four joints(Ab/duction, MCP, PIP, DIP)
%               'RRRp': planar finger with three joints(MCP, PIP, DIP; ABD joint is fixed)
%               'RRRs': spatial finger with three joints(Ab/duction, MCP, PIP; DIP joint is fixed)
%           link_len_vector:
%               vector of the length of the links that match the number
%               the finger_type (mdh: a)
%
%       Coordinates:   
%           Coord. World: (w) world coordinate. symmetrical for all classes
%           Coord. base:  original coordinate for the **Class Finger**. 
%                         Tranformation information regarding w: w_p_base & w_R_base
%           Coord. jonts: mdh related 
%           

% TODO: 
%       1. [01/23]: integrate transformation from W to Base in properties
%       and Jacobian.
%       
% 

%   functions:
%       obj.update_finger(q_a): 
%           obj.mdh_all, obj.mdh, obj.M_coupling, rst_model 
%       obj.update_finger_par_dyn:
%           obj.par_dyn_f
%       obj.update_list_contacts
%       obj.update_list_tendons
%       obj.update_rst_model
%   tendon related: 
%       add_tendon();
%       update_M_coupling()
%       set_tendon_par_MA_poly3(obj, tendon_index, joint_index, par)
%       
%       

classdef Finger < handle & matlab.mixin.Copyable
    %FINGER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        name                % [char] name of the finger
        type                % [char] finger type: 'RRRR', 'RRRp','RRRs'
        nj                  % [1] number of joints
        nja                 % [1] number of active joints    
        nl                  % [1] number of links
        nc                  % [1] number of contacts
        nt                  % [1] number of tendons
        list_joints         % [njx1] list of all joints
        list_links          % [nlx1] list of all links
        list_contacts       % [ncx1] list of all contacts
        list_tendons        % [ntx1] list of all tendons
        joint_act           % [njx1] logical vector: 1: active joint; 0: fixed joint 
        limits_q            % [njx6] min,max values of q,qd,qdd
        limits_ft           % [ntx2] min,max values of tendon forces
        limits_t_ma         % [ntx..] min,max values of tendon forces
        par_kin             % all kinematic parameters
        opt_pkin            % kinematic parameters for optimization
        
        w_p_base            % position of the base in world frame. Default: [0,0,0]'
        w_R_base            % rotation of the base in world frame. Default: non rotation
%         w_T_base   


    end
    
    properties (SetAccess = private)
        mdh                 % mdh parameters: alpha, a theta, d (with q)
        mdh_all
        mdh_ori             % mdh parameters: alpha, a theta, d (without q)
        rst_model           % robotic system toolbox model based on mdh. Default: colume Dataformat
        q_a                 % [njax1] unit rad
        q                   % [njx1] unit rad
        par_dyn_f           
        M_coupling          % [njxnt] coupling matrix of the finger
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
                link_name_i = strcat('Link',num2str(i));
                Link_new = Links(link_name_i,link_len_input(i), i);
                list_links = [list_links;Link_new];
            end
            obj.list_links = list_links;
            
            % generate joints 
            % TODO 
            list_joints = [];
            for i = 1:obj.nj
                Joint_name_i = strcat('Joints',num2str(i));
                Joint_new = Joints(Joint_name_i,i);
                list_joints = [list_joints;Joint_new];
            end
            obj.list_joints = list_joints;
            obj.nja = obj.nj;
            obj.joint_act = ones(obj.nj,1);% TODO:passive joints
            
            
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
            
            
            obj.limits_q = zeros(obj.nj,6);
            % init finger base position and orientation
            obj.w_p_base = [0;0;0];
            obj.w_R_base = eye(3);
            obj.q_a = zeros(obj.nja,1);
            obj.q = zeros(obj.nj,1);
            % update mdh parameters
            obj.par_dyn_f = struct();
            obj.par_dyn_f.g = [0;0;-9.81];
            
            
            % set mdh parameters
%             obj.set_mdh_parameters(mdh_matrix)
            
            % update dynamic parameters
            obj.update_finger_par_dyn;
            obj.update_joints_info;
            % update parameters w.r.t. joint angles
            obj.update_finger(obj.q_a);
            
            obj.update_rst_model;
            
        end
        
        function check_finger_properties(obj)
            % check if all the properties of the Finger class are in the
            % correct form
            % TODO 
           
        end
        
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
            % with q
            mdh_new = mdh_ori_new;
            mdh_new.theta = [obj.q;0];
            obj.mdh = mdh_new;
            obj.mdh_all = mdh_new;
            
            % update rsi model
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
        
        
        
        function update_finger(obj, q_a)
            % update mhl parameters and par_kin, opt_pkin, ...
            assert(length(q_a)== obj.nja, 'dimension of joint vector is incorrect!')
            
            q_a = reshape(q_a,[obj.nj,1]);
            obj.q = q_a; % update joint angle
            obj.q_a = q_a;
%             obj.q(obj.joint_act) = q_a; % update joint angle
            
            % update mdh parameters
%             [obj.mdh,obj.mdh_all, obj.mdh_ori] = get_finger_mdh(obj, obj.q_a);
            
            obj.mdh_all.theta = obj.mdh_ori.theta +  [obj.q;0];
            obj.mdh = obj.mdh_all;
            % update par_kin 
            
            % update all link properties
            for i = 1:obj.nl
                mdh_all_matrix = mdh_struct_to_matrix(obj.mdh_all, 1);
                b_T_i = T_mdh_multi(mdh_all_matrix(1:i,:));
                base_p_link_i = b_T_i(1:3,4);
                base_R_link_i = b_T_i(1:3,1:3);
                obj.list_links(i).update(base_p_link_i,base_R_link_i);
            end
            
            % update joints info
%             obj.update_joints_info;
            % update the list of tendons
            obj.update_M_coupling(obj.q_a); 
            
            % update tendon force limits
%             obj.update_tendon_force_limits;
            % update dynamic parameters
%             obj.update_finger_par_dyn;    
            % update rsi model
%             obj.update_rst_model;
        end
        
        % joint related functinos
        function update_joints_info(obj)
            % update joint information
            %   1. limits_q
            
%             q = obj.q_a;
            for i = 1:obj.nj
               obj.limits_q(i,1:2) = obj.list_joints(i).q_limits;
               obj.limits_q(i,3:4) = obj.list_joints(i).qd_limits;
               obj.limits_q(i,5:6) = obj.list_joints(i).qdd_limits;
               
               
            end 
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
        
%%%%%%%%%%%%%%%%%%%%%%% dynamic parameters
        function update_finger_par_dyn(obj)
            % update the finger dynamic parameters
            % TODO: finalize for all types
            
            mass_all = zeros(obj.nja+1,1); % first is base
            com_all = zeros(3,obj.nja+1);
            inertia_all = zeros(6,obj.nja+1);
            
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
            g_reshape = reshape(g,3,1);
            w_R_b = obj.w_R_base;
            obj.par_dyn_f.g = w_R_b'*g_reshape;
            
        end

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
        % to be replaced
%         function [mdh_reduced,mdh_all,mdh_ori] = get_finger_mdh(obj, q_a)
%             % get mdh parameter from the given joint angle
%             % update mhl parameters and par_kin, opt_pkin, ...
%             % TODO: [01/23] only theta is updated, others are fixed in some
%             % sense.
%             assert(length(q_a)== obj.nja, 'dimension of joint vector is incorrect!')
%             
%             q_a = reshape(q_a,[obj.nja,1]);
%             q_all = obj.q;
%             q_all(obj.joint_act) = q_a;
% 
%             % update mdh parameters 
%             mdh_all = struct;
%             mdh_all.alpha = [0;-pi/2;0;0;0];
%             mdh_all.a = [0;0;obj.list_links(1).Length;obj.list_links(2).Length;obj.list_links(3).Length];
%             mdh_all.theta = [q_all(1);q_all(2);q_all(3);q_all(4);0]; % 
%             mdh_all.d = [0;0;0;0;0];
%             mdh_tmp = mdh_all;
%             fieldname_mdh = fieldnames(mdh_tmp);
%             if strcmp(obj.type,'RRRR') 
%                 
%             elseif strcmp(obj.type,'RRRp') 
%                 for i = 1:length(fieldname_mdh)
%                     mdh_tmp.(fieldname_mdh{i}) = mdh_tmp.(fieldname_mdh{i})(2:end);  
%                 end
%             elseif strcmp(obj.type,'RRRs')
%                 for i = 1:length(fieldname_mdh)
%                     mdh_tmp.(fieldname_mdh{i}) = mdh_tmp.(fieldname_mdh{i})(1:end-1);  
%                 end
%             end
%             mdh_reduced = mdh_tmp;
%             mdh_ori = mdh_all;
%             mdh_ori.theta = [0;0;0;0;0];
%         end
        
        function set_base(obj, w_p_base, w_R_base)
            % set the position and orientation of the base in the world
            % frame
            obj.w_p_base = w_p_base;
            obj.w_R_base = w_R_base;
            obj.update_finger(obj.q_a);
            
        end
        
        function W_T_b = get_W_T_B(obj)
            % get the homogeneous transformation matrix from Base to W
            W_T_b = [obj.w_R_base,obj.w_p_base; 0 0 0 1];
        end
        
        function update_rst_model(obj)
            % build/update the rst model of the finger based on the mdh
            % parameters
            %
            % TODO: integrate the dynamic parameters of the links into
            % model
                         
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
            for i = 1:obj.nj
                % transfer inertia into bodyframe
                inertia_all_rst(:,i+1) = inertia_all(:,i+1) + ...
                    mass_all(i+1)*[com_all(2,i+1).^2+com_all(3,i+1).^2;com_all(1,i+1).^2+com_all(3,i+1).^2;com_all(1,i+1).^2+com_all(2,i+1).^2;...
                    -com_all(2,i+1)*com_all(3,i+1);-com_all(1,i+1)*com_all(3,i+1);-com_all(1,i+1)*com_all(2,i+1)];
            end

            % virtual first body:  from CS.base to CS.1
            bodyname = obj.list_links(1).name;
            body1 = rigidBody(bodyname);
            jointname = 'virtual_Base_joints';
            jnt1 = rigidBodyJoint(jointname,'revolute');          
            W_T_base = obj.get_W_T_B(); % World to Base 
%             T = T_mdh_multi(mdh_matrix_order_1(1,:));
            setFixedTransform(jnt1,W_T_base);
            body1.Joint = jnt1;
            body1.Mass = mass_all(2); % first body (virtual)
            body1.Inertia = inertia_all_rst(:,2);
            body1.CenterOfMass = com_all(:,2);          
            addBody(rst_model_tmp,body1,'base');
            bodyname_last = bodyname;
%             bodyname_last = 'base';
            for j = 2:obj.nl
                bodyname = obj.list_links(j).name;
                body1 = rigidBody(bodyname);
                jointname = obj.list_joints(j).name;
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
            bodyname = 'endeffector';
            body1 = rigidBody(bodyname);
            jointname = 'Virtual_fingertip';
            jnt1 = rigidBodyJoint(jointname,'fixed');          
            setFixedTransform(jnt1,mdh_matrix(end,:),'mdh');
            body1.Joint = jnt1;
            body1.Mass = 0;
            body1.Inertia = [0,0,0,0,0,0];
            body1.CenterOfMass = [0,0,0];
            addBody(rst_model_tmp,body1,bodyname_last);
            
            obj.rst_model = rst_model_tmp;
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
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % calculate kinematics 
        function [p_link_all_w,p_link_all_b] = get_p_all_links(obj)
            % get the Cartesian position of the base and link ends
            w_R_b = obj.w_R_base; 
            w_p_b = obj.w_p_base;
%             w_T_b = get_W_T_B(obj);
            p_link_all_b = zeros(3,obj.nl+1);
            p_link_all_w = zeros(3,obj.nl+1);
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
        
        function [w_p_ee,w_R_ee] = get_T_ee_w(obj)
            % get the Cartesian position of the base and link ends
            w_R_b = obj.w_R_base; 
            w_p_b = obj.w_p_base;
%             w_T_b = get_W_T_B(obj);
            w_R_ee = zeros(3,3);
            w_p_ee = zeros(3,1);
            % fingertip position
            mdh_all_matrix = mdh_struct_to_matrix(obj.mdh_all, 1);
            b_T_i = T_mdh_multi(mdh_all_matrix);
            w_p_ee = w_p_b + w_R_b * b_T_i(1:3,4);
            w_R_ee =  w_R_b * b_T_i(1:3,1:3);
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
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%% print finger
        function print_contact(obj) % 
            % plot 3d contact points 
            for i = 1:obj.nl
                contact_pos = obj.list_links(i).contacts(1).base_p;
                W_contact_pos = obj.w_R_base*contact_pos + obj.w_p_base;
                plot3(W_contact_pos(1),W_contact_pos(2),W_contact_pos(3),'*','Color', 'r', 'MarkerSize',10)
                hold on
            end
        end
        function print_finger(obj,varargin) % 
            % plot 3d contact points 
            % [06.09.2023] new version of plot is avaible: plot_finger
            if nargin == 1
                color = 'r';
                linewidth = 3;
                markersize = 5;
            elseif  nargin == 2
                color = varargin{1};
                linewidth = 3;
                markersize = 5;
            elseif  nargin == 3
                color = varargin{1};
                linewidth = varargin{2};
                markersize = 5;
            elseif  nargin == 4
                color = varargin{1};
                linewidth = varargin{2};
                markersize = varargin{3};
            end
            p_link_all_w_r = obj.get_p_all_links;
            plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color',color,...
                                    'LineWidth',linewidth,'MarkerSize',markersize);
            hold on
        end

        function plot_finger(obj,parameters) 
            % [06.09.2023] plot the finger with paramter structures
            % parameters.linecolor = 'r'
            %           .linewidth = 3
            %           .markersize = 5
            %           .axis_show = 1
            %           .linewidth = 3
            %           .linewidth = 3
            
            w_R_b = obj.w_R_base;
            w_p_b = obj.w_p_base;

            color = parameters.linecolor;
            linewidth = parameters.linewidth;
            markersize = parameters.markersize;

            p_link_all_w_r = obj.get_p_all_links;
            plot3(p_link_all_w_r(1,:)',p_link_all_w_r(2,:)',p_link_all_w_r(3,:)','o-','Color',color,...
                                    'LineWidth',linewidth,'MarkerSize',markersize);
            hold on
            if parameters.axis_show
                for i = 1:obj.nl
                    length_link_i = obj.list_links(i).Length;
                    W_p_link_i = w_p_b + w_R_b*obj.list_links(i).base_p;
                    W_p_link_z_i = 0.5*length_link_i*w_R_b*obj.list_links(i).base_R*[0;0;1];
                    quiver3(W_p_link_i(1),W_p_link_i(2),W_p_link_i(3),...
                            W_p_link_z_i(1),W_p_link_z_i(2),W_p_link_z_i(3),...
                            'Color','b','LineWidth',linewidth);
                    hold on
                end 

            end
        end

        function parstr = plot_parameter_init(obj)
            % init the parameter struct for plot_finger
            parstr = struct();
            parstr.linecolor = 'r';
            parstr.linewidth = 3;
            parstr.markersize = 5;
            parstr.axis_show = 1;
%             parstr.linecolor = 'r';
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % tendon related:
        function add_tendon(obj, name, routing)
            % add tendon to class
            % TODO: partition q & qa (current version: using q_a)
            tendon_tmp = Tendons(name, routing, obj.nt+1);
            tendon_tmp.init_tendon_par(obj.nj, obj.q_a)
            obj.nt = obj.nt + 1;
            obj.list_tendons = [obj.list_tendons;tendon_tmp];
            obj.update_tendon_ma_limits_from_joints;
            obj.update_M_coupling(obj.q_a);
            obj.update_tendon_force_limits;
        end
        
        
        
        function set_tendon_par_MA_poly3(obj, tendon_index, joint_index, par)
            
            assert(length(par)== 4, 'dimension of par_MA_poly3 is incorrect!')
            assert(tendon_index <= obj.nt, 'index of tendon is incorrect!')
            assert(joint_index <= obj.list_tendons(tendon_index).j_index, 'index of joint is incorrect!')
            obj.list_tendons(tendon_index).set_par_MA_poly3(joint_index, par, obj.q_a);
            obj.update_M_coupling(obj.q_a);
            
        end
        
        function update_tendon_ma_limits_from_joints(obj)
            for i = 1:obj.nt
                tendon_j_index_i = obj.list_tendons(i).j_index;
                ma_limits_i = zeros(tendon_j_index_i,2);
                for j = 1:tendon_j_index_i
                    ma_limits_i(j,:) = obj.list_tendons(i).routing(j)*obj.list_joints(j).momentarm_limits;
                    obj.list_tendons(i).update_ma_limits(j,ma_limits_i(j,:))
                    obj.limits_t_ma(i,2*j-1:2*j) = ma_limits_i(j,:);
                end
            end
        end
        
        function M_coupling = update_M_coupling(obj, q)
            % update M_coupling matrix w.r.t. current configuration
            % TODO: moment arm value as a function of q
            M_coupling_tmp = zeros(obj.nj,obj.nt);
            for i = 1:obj.nt
                obj.list_tendons(i).update_momentarm(q);
                M_coupling_tmp(:,i) = obj.list_tendons(i).routing.*obj.list_tendons(i).moment_arm_abs;
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
                M_coupling_i = abs(obj.M_coupling(i,:));
                M_coupling_i(M_coupling_i>ma_max_i) = ma_max_i;
                M_coupling_i(M_coupling_i<ma_min_i) = ma_min_i;
                obj.M_coupling(i,:) = sign(obj.M_coupling(i,:)).*M_coupling_i;
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
    end
end

