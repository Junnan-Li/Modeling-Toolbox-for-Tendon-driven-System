% Geometric Jacobian of the end-effector based on the jonit cofiguration
% reference coordinate: CS base of the finger
% 
% input:
%           obj
%           varargin
%               q:  [obj.nj,1] Jacobian with given q 
% 
% output:
%           J: [6xobj.nj] geometric jacobian
% 
% Source: 
%   "Robotics: Modeling, Planning, and Control" P111 - P113
% 
% Note: the same as J = Jacobian_geom_mdh(mdh,q)
% 
% Junnan Li, junnan.li@tum.de, 01.2023

function J = Jacobian_geom_b_end(obj,varargin)

mdh = mdh_struct_to_matrix(obj.mdh,1);

if nargin == 2
    q = varargin{1};
    assert(length(q)== obj.nja, 'dimension of joint vector is incorrect!')
    q = reshape(q,[obj.nja,1]);
    mdh(1:obj.nj,3) = q;
end
J = zeros(6,obj.nj);

if obj.kin_use_T
    b_T_all = obj.get_b_T_all_links;
    base_T_end = b_T_all(:,:,end);
    for i = 1:obj.nj
        base_T_i = b_T_all(:,:,i);
        b_R_i = base_T_i(1:3,1:3);
        i_T_end = base_T_i\base_T_end;
        i_p_end = i_T_end(1:3,4);
        J_tmp_p = cross(b_R_i*[0;0;1],b_R_i*i_p_end);
        J_tmp_w = b_R_i*[0;0;1];
        J(:,i) = [J_tmp_p;J_tmp_w];
    end
else
    base_T_end = T_mdh_multi(mdh);
    for i = 1:obj.nj
        base_T_i = T_mdh_multi(mdh(1:i,:));
        b_R_i = base_T_i(1:3,1:3);
        i_T_end = base_T_i\base_T_end;
        i_p_end = i_T_end(1:3,4);
        J_tmp_p = cross(b_R_i*[0;0;1],b_R_i*i_p_end);
        J_tmp_w = b_R_i*[0;0;1];
        J(:,i) = [J_tmp_p;J_tmp_w];
    end
end
end

