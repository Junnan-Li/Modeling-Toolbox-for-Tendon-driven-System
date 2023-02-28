% Geometric Jacobian of the end-effector based on the jonit cofiguration
% reference coordinate: CS base of the finger
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

function J = Jacobian_geom_b_end(obj,q)

if nargin == 1
    q = obj.q;
end
assert(length(q)== obj.nja, 'dimension of joint vector is incorrect!')
q = reshape(q,[obj.nja,1]);

num_active_joint = obj.nja;
% mdh_struct = obj.get_finger_mdh(q);
mdh = mdh_struct_to_matrix(obj.mdh,1);
J = zeros(6,num_active_joint);
base_T_end = T_mdh_multi(mdh);

for i = 1:num_active_joint
    
    base_T_i = T_mdh_multi(mdh(1:i,:));
    b_R_i = base_T_i(1:3,1:3);
    i_T_end = base_T_i\base_T_end;
    i_p_end = i_T_end(1:3,4);
    J_tmp_p = cross(b_R_i*[0;0;1],b_R_i*i_p_end);
    J_tmp_w = b_R_i*[0;0;1];
    J(:,i) = [J_tmp_p;J_tmp_w];
end

end

