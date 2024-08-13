% Translational Geometric Jacobian of the a given point at a given frame  
% 
% input:
%           q: 
% 
% output:
%           J_trans: [3xobj.nja] geometric jacobian
% 
% Source: 
%   "Robotics: Modeling, Planning, and Control" P111 - P113
% 
% Note: the same as J = Jacobian_geom_mdh(mdh,q)
% 
% Junnan Li, junnan.li@tum.de, 01.2023

function J_trans = Jacobian_geom_w_point(obj, index_link, w_p_point,varargin)

if nargin <= 3
    w_T_link_all = obj.get_w_T_links_inhand;
else
    w_T_link_all = varargin{1};
end
% assert(length(q)== obj.nj, '[Jacobian_geom_w_all_fingers]: dimension of joint vector is incorrect!')

index_base = [1,obj.index_q_b(end,end)];
index_finger = obj.index_q_f;

J = Jacobian_geom_T_hand(w_T_link_all, index_base, index_finger, index_link, w_p_point);
J_trans = J(1:3,:);
% obj.update_hand(q_init);

end