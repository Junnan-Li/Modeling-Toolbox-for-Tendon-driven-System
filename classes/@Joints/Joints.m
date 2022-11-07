%% Class of Joints:
% 
%       Create a Link:



classdef Joints
    %JOINTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name
        momentarm_limits
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
            obj.momentarm_limits = [0.02, 0.05];
            obj.w_p = [0,0,0]';
            obj.w_R = eye(3);
        end
        
%         function outputArg = method1(obj,inputArg)
%             %METHOD1 Summary of this method goes here
%             %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end

