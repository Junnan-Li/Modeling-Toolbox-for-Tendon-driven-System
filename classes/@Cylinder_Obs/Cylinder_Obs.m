classdef Cylinder_Obs < Obstacles
    
    properties (Access = public)
        axis
        radius
        length
    end
    properties (SetAccess = private)

    end
    
    methods
        function obj = Cylinder_Obs(name)
            % Obstacle initialize
            obj = obj@Obstacles(name)
            obj.axis = [0,0,1];
            obj.radius = 0;
            obj.length = 0;
        end

    end
end

