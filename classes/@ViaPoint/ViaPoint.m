%% Class of ViaPoint
% ViaPoint defines the origin, insertion, and pathway of the muscles. This object will also
% used in the obstacle-set algorithms of calculating moment arm at joints.
% ViaPoint is defined as a fixed point of a body frame 
%           


classdef ViaPoint < handle

    properties (Access = public)
        name                 % [char] name of ViaPoint
        link_p              % [3x1] position in the link frame
    end
    properties (SetAccess = private)
%         link_index
        link_name
        base_p          % position related to base frame, need be calculate with given link T matrix
    end

    methods
        function obj = ViaPoint(name, link_name, link_p)
            % ViaPoint initialize
            obj.name = name;
            obj.link_name = link_name;
%             obj.link_index = 0;
            obj.link_p = link_p;

        end

        function update_VP_pose(obj,base_p_link,base_R_link)
            % calculate the VP position in the base frame with given link
            % transformation matrix. 
            base_p_link = reshape(base_p_link,[3,1]);
            base_T_VP = [base_R_link,base_p_link;0 0 0 1];
            H_tmp = base_T_VP * [obj.link_p;1];
            obj.base_p = H_tmp(1:3);
        end
    end
end

