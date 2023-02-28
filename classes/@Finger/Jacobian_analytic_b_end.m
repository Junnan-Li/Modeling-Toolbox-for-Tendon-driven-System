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

num_active_joint = obj.nja;
% mdh_struct = obj.get_finger_mdh(q);
mdh = mdh_struct_to_matrix(obj.mdh_ori,1);

J = Jacobian_analytic_mdh(mdh,q);

end

