%% Class of Joints:
% 
%       Create a Link:



classdef Joints
    %JOINTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name
        momentarm_limits
        q_limits             %[2x1]: [min,max] joint angle
        qd_limits             %[2x1]: [min,max] joint angle
        qdd_limits             %[2x1]: [min,max] joint angle
        w_p
        w_R
    end
    properties (SetAccess = private)
        index  
        fixed
    end
    methods
        function obj = Joints(name,index)
            %JOINTS Construct an instance of this class
            obj.name = name;
            obj.index = index;
            obj.momentarm_limits = [0.005, 0.02];
            obj.w_p = [0,0,0]';
            obj.w_R = eye(3);
            obj.q_limits = [-10,100]*pi/180;
            obj.qd_limits = [-90,90]*pi/180;
            obj.qdd_limits = [-90,90]*pi/180;
            obj.fixed = 0;
        end
        
        function set_fixed(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.fixed = 1;
        end
    end
end

