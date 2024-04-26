%% Class of Links:
% 
%       Create a Link:





classdef Links < handle
    
    properties (Access = public)
        name
        Length              % [1] Length in m
        Width               % [1] Width in m
        Thick               % [1] Thick in m
        par_dyn             % [1] mass, [3x1] center of mass, [6x1] inertia
        stiffness
        nc
        contacts            % [1] num of viapoints
        viapoints
        nv                  % [1] num of viapoints
        

    end
    properties (SetAccess = private)
        index  
        base_p            % [3x1] b_p_i: position of the frame with respect to Finger base frame. 
        base_R            % [3x3] b_R_i: position of the frame with respect to Finger base frame. 
    end
    
    methods
        function obj = Links(name, length, index)
            %LINK initialize
            obj.name = name;
            obj.Length = length;
            % general value: 
            obj.Width = 0.03;
            obj.Thick = 0.03;
            obj.par_dyn = struct;
            obj.par_dyn.mass = 1;
            obj.par_dyn.com = 1/2 * [obj.Length,0,0]'; % middle point of the links
            obj.par_dyn.inertia = [1,1,1,0,0,0]'; % [xx yy zz yz xz xy]; related to center of mass
            obj.stiffness = 1;
            obj.index = index; 
            obj.contacts = [];
            obj.nc = 0;
            obj.viapoints = [];
            obj.nv = 0;
            obj.base_p = [0;0;0]; 
            obj.base_R = eye(3);
        end
        
        function update(obj,base_p,base_R)
            %update the link position and rotation based on the joint
            %config.
            assert(length(base_p) == 3, 'Function error: input dimension of link position is incorrect');
%             assert(size(base_R) == 3, 'Function error: input dimension of link position is incorrect');           
            obj.base_p = reshape(base_p,[3,1]);
            obj.base_R = base_R;
            obj.update_link_contacts
        end
        
        %% contacts
        function new_contact = add_contact_link(obj,name, link_p)
            %add contact point on the link
            % link_p: 
            %       local position of the contact point 
            assert(length(link_p) == 3, 'Function error: input dimension of contact position is incorrect');
            link_p = reshape(link_p,[3,1]); % columewise
            
            contact_name = name;
            new_contact = Contacts(contact_name,obj.index, link_p);
            
            obj.contacts = [obj.contacts;new_contact];
            obj.nc = length(obj.contacts);      
            obj.update_link_contacts; % 
        end
        
        function delete_all_contacts_link(obj)
            % delete all contacts of the link
            obj.contacts = [];
            obj.update_link_contacts();
        end
        
        function delete_one_contact(obj,index)
            % delete all contacts of the finger
            num_c = obj.nc;
            if index < num_c
                obj.contacts(index) = [];
            else
                disp('Can not delete the Contact: the index is out of the range')
            end
            obj.update_list_contacts;
        end
        
        function update_link_contacts(obj)
            % update obj.nc link.Contact.base_R & base_p
            
            obj.nc = length(obj.contacts);
            if obj.nc ~=0
                for i = 1:obj.nc
                    obj.contacts(i).update_contact_pose(obj.base_p,obj.base_R)
                end
            end
        end
        
        %% dynamics
        function set_mass(obj, mass)
            % set dynamic parameters
            obj.par_dyn.mass = mass;
        end
        
        function set_inertia(obj, inertia)
            % set dynamic parameters
            assert(length(inertia) == 6,'Input error: inertia tensor size is wrong')
            obj.par_dyn.inertia = reshape(inertia,6,1);
        end
        
        function set_com(obj, com)
            % set dynamic parameters
            assert(length(com) == 3,'Input error: CoM size is wrong')
            obj.par_dyn.com = reshape(com,3,1);
        end
        
        %% Viapoints

        function new_viapoint = add_viapoint_link(obj, name, link_p)
            % add viapoint on the link
            % link_p: 
            %       local position of the viapoint 
            assert(length(link_p) == 3, 'Function error: input dimension of contact position is incorrect');
            link_p = reshape(link_p,[3,1]); % columewise
            
            new_viapoint = ViaPoint(name, obj.index, link_p);
            obj.viapoints = [obj.viapoints;new_viapoint];
            obj.nv = length(obj.viapoints);      
            obj.update_link_viapoints; % 
        end
        
    end
end

