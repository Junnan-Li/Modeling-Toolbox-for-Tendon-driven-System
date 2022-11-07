%% Class of Finger:
% 
%       Create a Finger:
%           finger = Finger(finger_type, link_len_vector)
%           
%           finger_type: 
%               'RRRR': general finger with four joints(Ab/duction, MCP, PIP, DIP)
%               'RRRp': planar finger with three joints(MCP, PIP, DIP)
%               'RRRs': spatial finger with three joints(Ab/duction, MCP, PIP, DIP)
%           link_len_vector:
%               vector of the length of the links that match the number
%               the finger_type

    




classdef Finger < handle
    %FINGER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        name
        nj                  % number of joints
        nl
        nc
        nt
        list_joints
        list_links
        list_contacts
        list_tendons
        joint_act           % [njx1] binary vector: 1: active joint; 0: fixed joint 
        
        pkin                % update finger
        opt_pkin
        
        w_p_base                
        w_R_base
        w_T_base   
    end
    
    properties (SetAccess = private)
        mdh
    end
    
    methods
        function obj = Finger(name, varargin)
            % finger_type: 'RRRR', 'RRRs', 'RRRp'
            assert(nargin>3 || ischar(name), 'Finger class input not correct!')
 
            
            if nargin == 2
                finger_type = varargin{1};
                link_len_vector = varargin{2};
            elseif nargin == 2
                finger_type = varargin{1};
                link_len_vector = 0.05 * ones(count(finger_type,'R'),1);
            elseif  nargin == 1
                finger_type = 'RRRR';
                link_len_vector = 0.05 * ones(count(finger_type,'R'),1);
            end
            
            obj.name = name;
            obj.nj = count(finger_type,'R');
            obj.nl = 3;
            
            % create links
            PP = Links('PP',0.07, 1);
            MP = Links('MP',0.07, 2);
            DP = Links('DP',0.07, 3);
            obj.list_links = [PP;MP;DP];
            
            % create joints
            ABD = Joints('ABD',1);
            MCP = Joints('MCP',2);
            PIP = Joints('PIP',3);
            DIP = Joints('DIP',4);
            obj.list_joints = [ABD;MCP;MP;DP];
            obj.joint_act = [];
            if strcmp(finger_type,'RRRR') 
                obj.joint_act = [1,1,1,1]';
            elseif strcmp(finger_type,'RRRp') 
                obj.joint_act = [0,1,1,1]';
            elseif strcmp(finger_type,'RRRs')
                obj.joint_act = [1,1,1,0]';
            end
            
            
            
            
            
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

