% Analytical Jacobian of the end-effectors of one finger 
% 
% input:
%           finger_index: 
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

function J_ana = Jacobian_ana_w_one_finger(obj,finger_index,q)

if nargin == 2
    q = obj.q;
end
assert(length(q)== obj.nj, '[Jacobian_geom_w_one_finger]: dimension of joint vector is incorrect!')
assert(finger_index <= obj.nf, '[Jacobian_geom_w_one_finger]: dimension of joint vector is incorrect!')

q_init = obj.q;
q = reshape(q,[obj.nj,1]);
obj.update_hand(q);

% w_T_fingeri_end
w_T_fi_end = obj.list_fingers(finger_index).get_w_T_ee_inhand;
w_R_fi_end = w_T_fi_end(1:3,1:3);

J = Jacobian_geom_w_one_finger(obj,finger_index,q);
J_ana = blkdiag(eye(3),w_R_fi_end)*J;

obj.update_hand(q_init);
end