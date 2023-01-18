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




classdef Finger < handle
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

        par_kin             % all kinematic parameters
        opt_pkin            % kinematic parameters for optimization
        
        w_p_base            % position of the base in world frame. Default: [0,0,0]'
        w_R_base            % rotation of the base in world frame. Default: non rotation
%         w_T_base   


    end
    
    properties (SetAccess = private)
        mdh                 % mdh parameters: alpha, a theta, d
        mdh_all
        rst_model           % robotic system toolbox model based on mdh. Default: colume Dataformat
        q_a                 % [njax1]
        q                   % [njx1]
        par_dyn_f           
    end
    
    methods
        function obj = Finger(name, varargin)
            % finger_type: 'RRRR', 'RRRs', 'RRRp'
            assert(nargin>3 || ischar(name), 'Finger class input not correct!')
            
            if nargin == 3
                obj.type = varargin{1};
                link_len_vector = varargin{2};
            elseif nargin == 2
                obj.type = varargin{1};
                link_len_vector = 0.05 * ones(count(obj.type,'R'),1);
            elseif  nargin == 1
                obj.type = 'RRRR';
                link_len_vector = 0.05 * ones(count(obj.type,'R'),1);
            end
            
            obj.name = name;
            %             obj.nj = count(obj.type,'R');
            obj.nj = 4; % default 4 joints
            obj.nl = 3;
            
            % create links
            PP = Links('PP',link_len_vector(1), 1);
            MP = Links('MP',link_len_vector(2), 2);
            DP = Links('DP',link_len_vector(3), 3);
            obj.list_links = [PP;MP;DP];
            
            % create joints
            % all the finger has 4 joints 
            ABD = Joints('ABD',1);
            MCP = Joints('MCP',2);
            PIP = Joints('PIP',3);
            DIP = Joints('DIP',4);
            obj.list_joints = [ABD;MCP;PIP;DIP];
            obj.joint_act = [];
            
            if strcmp(obj.type,'RRRR') 
                obj.joint_act = logical([1,1,1,1]');
                obj.nja = 4;
            elseif strcmp(obj.type,'RRRp') 
                obj.joint_act = logical([0,1,1,1]');
                obj.list_joints(1).set_fixed;
                obj.nja = 3;
            elseif strcmp(obj.type,'RRRs')
                obj.joint_act = logical([1,1,1,0]');
                obj.list_joints(4).set_fixed;
                obj.nja = 3;
            end
            
            % init list of contact and tendon
            % default contacts in the middle of each links
            obj.nc = 0;
            obj.list_contacts = [];
            
            obj.nt = 0;
            obj.list_tendons = [];
            
            % init finger base position and orientation
            obj.w_p_base = [0;0;0];
            obj.w_R_base = eye(3);
            obj.q_a = zeros(obj.nja,1);
            obj.q = zeros(obj.nj,1);
            % update mdh parameters
            obj.par_dyn_f = struct();
            obj.par_dyn_f.g = [0;0;-9.81];
            
            obj.update_finger(obj.q_a);
            
            % update dynamic parameters
            
            
        end
        
        function check_finger_properties(obj)
            % check if all the properties of the Finger class are in the
            % correct form
            % TODO 
            
            
            
        end
        
        function update_finger(obj, q_a)
            % update mhl parameters and par_kin, opt_pkin, ...
            assert(length(q_a)== obj.nja, 'dimension of joint vector is incorrect!')
            
            q_a = reshape(q_a,[obj.nja,1]);
            
            obj.q_a = q_a; % update joint angle
            obj.q(obj.joint_act) = q_a; % update joint angle
            
            % update mdh parameters
            [obj.mdh,obj.mdh_all] = get_finger_mdh(obj, obj.q_a);
            
            % update par_kin 
            
            % update all link properties
            % TODO
            for i = 1:obj.nl
                mdh_all_matrix = mdh_struct_to_matrix(obj.mdh_all, 1);
                b_T_i = T_mdh_multi(mdh_all_matrix(1:i+1,:));
                base_p_link_i = b_T_i(1:3,4);
                base_R_link_i = b_T_i(1:3,1:3);
                obj.list_links(i).update(base_p_link_i,base_R_link_i);
            end
            
            % update dynamic parameters
            obj.update_finger_par_dyn;     
            % update rsi model
            obj.update_rst_model;
            
            
        end
        
        function update_finger_par_dyn(obj)
            % update the finger dynamic parameters
            % TODO: finalize for all types
            
            mass_all = zeros(obj.nja+1,1); % first is base
            com_all = zeros(3,obj.nja+1);
            inertia_all = zeros(6,obj.nja+1);
            if strcmp(obj.type,'RRRR')
                for i = 2:1:obj.nja
                    mass_all(i+1) = obj.list_links(i-1).par_dyn.mass;
                    com_all(:,i+1) = obj.list_links(i-1).par_dyn.com;
                    inertia_all(:,i+1) = obj.list_links(i-1).par_dyn.inertia;
                end
            end
            obj.par_dyn_f.mass_all = mass_all;
            obj.par_dyn_f.com_all = com_all;
            obj.par_dyn_f.inertia_all = inertia_all;
            
        end
        
        function [mdh_reduced,mdh_all] = get_finger_mdh(obj, q_a)
            % get mdh parameter from the given joint angle
            % update mhl parameters and par_kin, opt_pkin, ...
            % TODO: [01/23] only theta is updated, others are fixed in some
            % sense.
            assert(length(q_a)== obj.nja, 'dimension of joint vector is incorrect!')
            
            q_a = reshape(q_a,[obj.nja,1]);
            q_all = obj.q;
            q_all(obj.joint_act) = q_a;

            % update mdh parameters 
            mdh_all = struct;
            mdh_all.alpha = [0;-pi/2;0;0;0];
            mdh_all.a = [0;0;obj.list_links(1).Length;obj.list_links(2).Length;obj.list_links(3).Length];
            mdh_all.theta = [q_all(1);q_all(2);q_all(3);q_all(4);0]; % 
            mdh_all.d = [0;0;0;0;0];
            mdh_tmp = mdh_all;
            fieldname_mdh = fieldnames(mdh_tmp);
            if strcmp(obj.type,'RRRR') 
                
            elseif strcmp(obj.type,'RRRp') 
                for i = 1:length(fieldname_mdh)
                    mdh_tmp.(fieldname_mdh{i}) = mdh_tmp.(fieldname_mdh{i})(2:end);  
                end
            elseif strcmp(obj.type,'RRRs')
                for i = 1:length(fieldname_mdh)
                    mdh_tmp.(fieldname_mdh{i}) = mdh_tmp.(fieldname_mdh{i})(1:end-1);  
                end
            end
            mdh_reduced = mdh_tmp;
        end
        
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
             
            mdh_matrix = mdh_struct_to_matrix(obj.mdh,2); % mdh order: a,alpha,d,theta
            
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
                inertia_all_rst(:,i+1) = inertia_all(:,i+1) + ...
                    mass_all(i+1)*[com_all(2,i+1).^2+com_all(3,i+1).^2;com_all(1,i+1).^2+com_all(3,i+1).^2;com_all(1,i+1).^2+com_all(2,i+1).^2;...
                    -com_all(2,i+1)*com_all(3,i+1);-com_all(1,i+1)*com_all(3,i+1);-com_all(1,i+1)*com_all(2,i+1)];
            end

            % virtual first body:  from CS.base to CS.1
            bodyname = 'virtual_Base';
            body1 = rigidBody(bodyname);
            jointname = 'MCP_abd';
            jnt1 = rigidBodyJoint(jointname,'revolute');          
            W_T_base = obj.get_W_T_B(); % World to Base 
            setFixedTransform(jnt1,W_T_base);
            body1.Joint = jnt1;
            body1.Mass = mass_all(2); % first body (virtual)
            body1.Inertia = inertia_all_rst(:,2);
            body1.CenterOfMass = com_all(:,2);          
            addBody(rst_model_tmp,body1,'base');
            bodyname_last = bodyname;
            
            for j = 1:obj.nl
                bodyname = obj.list_links(j).name;
                body1 = rigidBody(bodyname);
                jointname = obj.list_joints(j+1).name;
                jnt1 = rigidBodyJoint(jointname,'revolute');
                setFixedTransform(jnt1,mdh_matrix(j+1,:),'mdh');
                body1.Joint = jnt1;
                body1.Mass = mass_all(j+2); % first body (virtual)
                body1.Inertia = inertia_all_rst(:,j+2);
                body1.CenterOfMass = com_all(:,j+2);
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
        
        
        function print_contact(obj)
            % plot 3d contact points 
            for i = 1:obj.nl
                contact_pos = obj.list_links(i).contacts(1).base_p;
                W_contact_pos = obj.w_R_base*contact_pos + obj.w_p_base;
                plot3(W_contact_pos(1),W_contact_pos(2),W_contact_pos(3),'*','Color', 'r', 'MarkerSize',10)
                hold on
            end
            
        end
        
        
        
    end
end

