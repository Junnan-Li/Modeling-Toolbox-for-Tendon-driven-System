%% Class of Links:
% 
%       Create a Link:





classdef Links
    
    properties (Access = public)
        name
        length
        width
        thick
        par_dyn             % mass, center of mass, inertia
        stiffness

    end
    properties (SetAccess = private)
        index  
    end
    
    methods
        function obj = Links(name, length, index)
            %LINK initialize
            obj.name = name;
            obj.length = length;
            % general value: 
            obj.width = 0.03;
            obj.thick = 0.03;
            obj.par_dyn = struct;
            obj.par_dyn.mass = 1;
            obj.par_dyn.com = 1/2 * [obj.length,obj.thick,obj.width];
            obj.par_dyn.inertia = [1,1,1,0,0,0]; % [xx yy zz xy xz yz];
            obj.stiffness = 1;
            obj.index = index; 
        end
        
%         function outputArg = method1(obj,inputArg)
%             %METHOD1 Summary of this method goes here
%             %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end

