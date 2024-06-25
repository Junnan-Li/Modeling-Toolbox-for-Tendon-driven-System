%% Class of ElasticMuscle:

classdef ElasticMuscle < Muscles

    properties (Access = public)
        length
        slack_length
        optimal_length
        stiffness
        damping
        MIF                     % [1] maximal isometric force
    end
    properties (SetAccess = private)
        velocity            % [1] number of joints from finger.nj
    end
    
    methods
        function obj = ElasticMuscle(name)
            % Muscle initialize
            obj.name = name;
%             obj.nj_finger = 0;
            obj.list_vp = [];
            obj.n_vp = 0; %
            obj.optimal_length = [];
            obj.slack_length = [];
            obj.stiffness = 0; %
            obj.damping = 0; %
            obj.MIF = 0; %
        end

        function f = get_muscle_force(obj)
            % 
            delta_l = obj.length - obj.slack_length;
            delta_v = -obj.velocity;
            if delta_l < 0
                f = 0;
            else
                f = obj.stiffness*delta_l + obj.damping*delta_v;
            end
            if f < 0
                disp("ElasticMuscle: f is negative!")            
            end
        end
        
        function set_muscle_slack_length(obj, l)
            % 
            obj.slack_length = l;
        end
        function set_muscle_optimal_length(obj, l)
            % 
            obj.optimal_length = l;
        end
        function set_muscle_stiffness(obj, k)
            % 
            obj.stiffness = k;
        end
        function set_muscle_damping(obj, d)
            % 
            obj.damping = d;
        end
        function set_muscle_velocity(obj, v)
            % 
            obj.velocity = v;
        end

    end
end

