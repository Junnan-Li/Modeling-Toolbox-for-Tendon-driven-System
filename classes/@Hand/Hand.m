%% Class of Hand:
% 



classdef Hand < handle & matlab.mixin.Copyable
    %FINGER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        name                % [char] name of the finger


    end
    
    properties (SetAccess = private)

    end
    
    methods
        function obj = Hand(name, varargin)
            % input syntax with priority:
            %   1. mdh struct: [alpha, a theta, d] without q
            %   2. finger_type: 
            %
            
            
            obj.name = name;
            obj.mdh_ori = [];
            
            % Finger constructor input
            ip = inputParser;
            %    validScalarPosNum = @(x) isnumeric(x) && isscalar(x) && (x > 0);
            ip.addParameter('mdh',[]);
            ip.addParameter('type',[]);
            ip.addParameter('l_links',[]);
            parse(ip,varargin{:});
            p = ip.Results;
            
            if ~isempty(p.mdh)
                obj.type = 'customized';
                obj.mdh_ori = p.mdh;
                obj.nj = length(obj.mdh_ori.a)-1; %
                obj.nl = obj.nj;
                link_len_input = obj.mdh_ori.d;
            elseif ~isempty(p.type)
                type_input_split = split(p.type,'_');
                if strcmp(type_input_split{1},'R')
                    obj.type = p.type;
                    obj.mdh_ori = p.mdh; % empty 
                    link_len_input = p.l_links;
                    obj.nj = count(type_input_split{2},'R');
                    obj.nl = obj.nj;
                elseif strcmp(type_input_split{1},'H') % TODO
                    obj.type = p.type;
                    obj.mdh_ori = p.mdh;
                    link_len_input = p.l_links;
                    obj.nj = 4;
                    obj.nl = 4;
                end
            end
            
            
        end
    end
end

