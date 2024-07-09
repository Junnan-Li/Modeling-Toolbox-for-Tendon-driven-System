%% Class of Obstacles

% Comment: 
%       properties:
%           name:    name of the muscle
%           links:   the link that obstacle attached 
% 
%       functions: 
%           

classdef Obstacles < handle
    
    properties (Access = public)
        name                 % [char] name of tendon
    end
    properties (SetAccess = private)

    end
    
    methods
        function obj = Muscles(name)
            % Obstacle initialize
            obj.name = name;
        end


        

    end
end

