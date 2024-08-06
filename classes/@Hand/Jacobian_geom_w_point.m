% Geometric Jacobian of the a given point at a given frame  
% 
% input:
%           q: 
% 
% output:
%           J: [6xobj.nja] geometric jacobian
% 
% Source: 
%   "Robotics: Modeling, Planning, and Control" P111 - P113
% 
% Note: the same as J = Jacobian_geom_mdh(mdh,q)
% 
% Junnan Li, junnan.li@tum.de, 01.2023

function J = Jacobian_geom_w_point(obj, index_link, i_p_point,varargin)

if nargin <= 3
    q = obj.q;
else
    q = varargin{1};
end
assert(length(q)== obj.nj, '[Jacobian_geom_w_all_fingers]: dimension of joint vector is incorrect!')
q_init = obj.q;
q = reshape(q,[obj.nj,1]);
obj.update_hand(q);

w_T_all = obj.get_w_T_links_inhand;
index_base = [1,obj.index_q_b(end,end)];
index_finger = obj.index_q_f;

J = Jacobian_geom_T_hand(w_T_all, index_base, index_finger, index_link, i_p_point);

obj.update_hand(q_init);

end