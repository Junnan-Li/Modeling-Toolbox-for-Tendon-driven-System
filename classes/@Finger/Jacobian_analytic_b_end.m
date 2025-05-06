% Analytical Jacobian of the end-effector based on the jonit cofiguration
% reference coordinate: CS base of the finger
% 
% input:
%           q: 
% 
% output:
%           J: [6xobj.nja] Analytical jacobian
% 
% Source: 
%   "Robotics: Modeling, Planning, and Control" P111 - P113
% 
% Note: the same as J = Jacobian_geom_mdh(mdh,q)
% 
% Junnan Li, junnan.li@tum.de, 01.2023

function J = Jacobian_analytic_b_end(obj,q)

if nargin == 1
    q = obj.q;
end
assert(length(q)== obj.nja, 'dimension of joint vector is incorrect!')
q = reshape(q,[obj.nja,1]);

% num_active_joint = obj.nja;
% mdh_struct = obj.get_finger_mdh(q);

if obj.kin_use_T
    J_geo_b = obj.Jacobian_geom_b_end;
    b_T_link = obj.get_b_T_all_links;
    b_R_end = b_T_link(1:3,1:3,end);
    J = blkdiag(eye(3),b_R_end)*J_geo_b;
else
    mdh = mdh_struct_to_matrix(obj.mdh_ori,1);

    J = Jacobian_analytic_mdh(mdh,q);
end
end

