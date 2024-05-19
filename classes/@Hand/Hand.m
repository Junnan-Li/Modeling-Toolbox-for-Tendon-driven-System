%% Class of Hand:
% 



classdef Hand < handle & matlab.mixin.Copyable
    %FINGER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        name                % [char] name of the finger
        nf                  % [1] number of fingers
        nj                  % [1] number of joints
        nja                 % [1] number of active joints    
        nl                  % [1] number of links
        nc                  % [1] number of contacts
        nt                  % [1] number of tendons
        nmus                % [1] number of muscles
        nvia                % [1] number of viapoints
        list_fingers        % [] list of all fingers
        list_joints         % [njx1] list of all joints
        list_links          % [nlx1] list of all links
        list_contacts       % [ncx1] list of all contacts
        list_tendons        % [ntx1] list of all tendons
        list_muscles        % [ntx1] list of all muscles
        list_viapoints      % [ntx1] list of all viapoints
        
        topology
    end
    
    properties (SetAccess = private)

    end
    
    methods
        function obj = Hand(name)
            % 

            % init lists      
            obj.nc = 0;
            obj.list_contacts = [];
            obj.nt = 0;
            obj.list_tendons = [];
            obj.limits_t_ma = [];
            
            obj.nmus = 0;
            obj.list_muscles = [];   

            obj.nvia = 0;
            obj.list_viapoints = [];
        end


        function add_finger(obj, finger)
            % add finger to hand  
            assert(isa(finger,'Finger'), '[add_finger] input is not a Finger object!');


        end
    end
end

