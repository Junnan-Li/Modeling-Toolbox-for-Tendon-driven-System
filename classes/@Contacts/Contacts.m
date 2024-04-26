%% Class of Contacts:
% 
%       Create a Contact:




classdef Contacts < handle
    
    properties (Access = public)
        name
        link_p          % [3x1] position related to link frame
        link_R          % [3x3] rotation matrix related to link frame
        type_friction
        dof
        par
        

    end
    properties (SetAccess = private)
        link_index  
        base_p          % position related to base frame
        base_R          % rotation matrix related to base frame
    end
    
    methods
        function obj = Contacts(name, link_index, link_p)
            %Contact initialize
            
            obj.name = name;
            obj.link_index = link_index;
            obj.link_p = link_p;
            obj.link_R = euler2R_XYZ([-pi/2 0 0]); % default frame is z axis point to the environment 
            obj.base_p = [];
            obj.base_R = eye(3);
            obj.type_friction = 'hard';
            obj.dof = [];
            obj.par = struct;      
        end
        
        function update_contact_pose(obj,base_p_link,base_R_link)
            
            base_p_link = reshape(base_p_link,[3,1]);
            base_T_contact = [base_R_link,base_p_link;0 0 0 1];
            H_tmp = base_T_contact * [obj.link_p;1];
            
            obj.base_p = H_tmp(1:3);
            obj.base_R = base_R_link * obj.link_R;
        end

    end
end

