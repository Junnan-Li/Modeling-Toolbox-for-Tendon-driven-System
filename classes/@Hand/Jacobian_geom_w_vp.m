% Geometric Jacobian of the a given point at a given frame  
% 
% input:
%           vp_obj: viapoint object 
%           varargin:
%               q: calculate the Jacobian of given configuration
% 
% output:
%           J: [3xobj.nja] translational geometric jacobian
% 
% Source: 
%   "Robotics: Modeling, Planning, and Control" P111 - P113
% 
% Note: the same as J = Jacobian_geom_mdh(mdh,q)
% 
% Junnan Li, junnan.li@tum.de, 01.2023

function J_tran = Jacobian_geom_w_vp(obj,varargin)

if nargin < 2
    vp_obj = obj.list_viapoints;
    q = obj.q;
elseif nargin < 3
    vp_obj = varargin{1};
    q = obj.q;
else
    vp_obj = varargin{1};
    q = varargin{2};
    q = reshape(q,[obj.nj,1]);
    obj.update_hand(q);
    q_init = obj.q;
end
assert(length(q)== obj.nj, '[Jacobian_geom_w_all_fingers]: dimension of joint vector is incorrect!')
q = reshape(q,[obj.nj,1]);
% obj.update_hand(q);

w_T_all = obj.get_w_T_links_inhand;
index_base = [];
index_finger = [];
if obj.njb ~= 0
    index_base = [1,obj.index_q_b(end,end)];
end
if obj.njf ~= 0
    index_finger = obj.index_q_f;
end
num_vp = length(vp_obj);
J_tran = nan(3*num_vp,obj.nj);


for i = 1:num_vp
    vp_obj_i = vp_obj(i);
    index_link = vp_obj_i.Link.index_inhand;
    w_T_link = vp_obj_i.Link.w_T_Link_inhand;
    i_p_point = T_p31(inv(w_T_link), vp_obj_i.w_p_VP_inhand);

    J = Jacobian_geom_T_hand(w_T_all, index_base, index_finger, index_link, vp_obj_i.w_p_VP_inhand);
    J_tran(3*i-2:3*i,:) = J(1:3,:);
end
if nargin > 2
    obj.update_hand(q_init);
end
end