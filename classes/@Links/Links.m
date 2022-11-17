%% Class of Links:
% 
%       Create a Link:





classdef Links < handle
    
    properties (Access = public)
        name
        Length
        Width
        Thick
        par_dyn             % mass, center of mass, inertia
        stiffness
        nc
        contacts
        

    end
    properties (SetAccess = private)
        index  
        base_p
        base_R
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
            obj.par_dyn.com = 1/2 * [obj.Length,obj.Thick,obj.Width];
            obj.par_dyn.inertia = [1,1,1,0,0,0]; % [xx yy zz xy xz yz];
            obj.stiffness = 1;
            obj.index = index; 
            obj.contacts = [];
            obj.nc = 0;
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
            obj.update_all_contacts
        end
        
        function add_contact(obj,link_p)
            %add contact point on the link
            assert(length(link_p) == 3, 'Function error: input dimension of contact position is incorrect');
            link_p = reshape(link_p,[3,1]); % columewise
            
            contact_name = ['contact_' num2str(obj.index) '_' num2str(obj.nc+1)];
            new_contact = Contacts(contact_name,obj.index, link_p);
            
            obj.contacts = [obj.contacts;new_contact];
            obj.nc = length(obj.contacts);      
            obj.update_all_contacts;
        end
        
        function update_all_contacts(obj)
            % update the Link Class property: Links.nc 
            if obj.nc ~=0
                for i = 1:obj.nc
                    obj.contacts(i).update_contact_pose(obj.base_p,obj.base_R)
                end
            end
        end
        
        
    end
end

